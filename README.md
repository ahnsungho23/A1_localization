# A1 localization workspace

ROS 2 Humble 기반 A1 실차 localization workspace다. Hesai PandarXT와 MOLA-LO로
LiDAR 지도를 만들고 그 지도에서 위치를 추정하며, Applanix GNSS/INS를 REP-105
구조(`map -> odom`은 MOLA, `odom -> base_link`는 INS)로 결합한다.
`localization_odom_merger`가 두 결과를 합쳐 제어기용 100 Hz
`/localization/odom`, `/localization/accel`을 발행한다. MOLA-LO 자체의 pose
추정에는 GNSS/IMU를 사용하지 않는다.

전체 구조는 [architecture](docs/architecture.md), 제어기 출력 형식은
[controller interface](docs/controller_interface.md)를 본다.

## 지원 환경

- Ubuntu 22.04
- ROS 2 Humble

## 현재 상태

실차 하드웨어(PandarXT, Applanix POS LVX)로 bag을 녹화해 확인한 항목:

- `/lidar_points`: `sensor_msgs/msg/PointCloud2`, 10 Hz, frame `hesai_lidar`,
  width `128000`, point step `26`, fields `x`, `y`, `z`, `intensity`, `ring`,
  `timestamp`
- `/lidar_packets_loss`: 손실 0
- Applanix: `/gsof_client/*` 100 Hz(GSOF #49), GSOF #50 5 Hz
- MOLA-LO: 2026-10-08 mapping bag 두 개(약 2.6 km, 2.9 km)를 끝까지 추적
- `localization_odom_merger`: 같은 bag 재생으로 RTK fixed 주행 중 LO 대비
  위치 오차 약 7~9 cm rms, 정차 중 GNSS 끌림(최대 6.9 m)을 stationary hold로 제거

아직 실차에서 확인하지 않은 항목:

- 현재 merger(정차 hold, heading 출력)의 실시간 동작
- Applanix 장착각 변경 후 `/a1_gnss/odom` 속도·가속도 축(직진 시 vx ≈ 속도, vy ≈ 0)
- `time_source: "now"` 설정에서 LiDAR와 GNSS 시각 정렬
- PTP lock

PandarXT의 `/lidar_imu`는 publisher만 생기고 메시지는 발행되지 않으므로 사용하지
않는다. MOLA-LO는 LiDAR registration에서 추정한 twist로 scan deskew를 수행한다.

## 저장소 받기

외부 드라이버와 MOLA가 submodule이므로 recursive clone을 사용한다.

```bash
git clone --recurse-submodules git@github.com:ahnsungho23/A1_localization.git a1_localization_ws
cd a1_localization_ws
```

이미 일반 clone을 했다면 다음 명령으로 submodule을 초기화한다.

```bash
git submodule update --init --recursive
```

## 호스트 네트워크 설정

다음 명령은 LiDAR UDP를 수신하는 **각 호스트 PC**에서 실행한다. `eno1`은 예시이므로 실제 Ethernet 인터페이스명으로 바꾼다. 대상 연결을 올리는 동안 Ethernet이 잠시 끊길 수 있으며, 해당 인터페이스를 사용하는 SSH 세션도 종료될 수 있다.

```bash
sudo ./tools/host/setup_lidar_host.sh eno1
```

기본 호스트 주소는 `192.168.1.100/24`다. 여러 PC를 센서 네트워크에 동시에 연결한다면 각 호스트에 서로 다른 주소를 지정한다.

```bash
sudo ./tools/host/setup_lidar_host.sh enp3s0 192.168.1.101/24
```

LiDAR 내부 설정, PTP 조건 및 receive-buffer 해석은 [PandarXT setup reference](docs/setup/pandarxt_setup.md)를 참고한다.

## 의존성 설치와 빌드

```zsh
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.zsh
```

MOLA 수정판(`mola_metric_maps` 3.1.1)도 이 workspace에서 함께 빌드되므로 별도
overlay를 source할 필요가 없다. 자세한 내용은
[MOLA-LO installation](docs/setup/mola_installation.md).

## 차량 실행

LiDAR 드라이버, Applanix GNSS/INS, 차량 TF, MOLA, `/localization/*` 출력과
RViz를 한 번에 실행한다. 항상 LiDAR와 GNSS/INS를 함께 사용하며, LiDAR만으로
실행하는 모드는 없다.

모든 터미널에서 먼저 실행한다. `WS`는 workspace 경로, `MAP_DIR`은 지도 폴더이며
**모든 터미널에서 같은 값**이어야 한다.

```bash
source /opt/ros/humble/setup.bash
WS=$HOME/git/A1_localization                 # 이 PC의 workspace 경로
source $WS/install/setup.bash
cd $WS
MAP_DIR=$WS/maps/track_20261104              # 지도 폴더 (mapping: 새 이름, 위치 추정: 실제 폴더 이름)
```

### Mapping (georef 가능한 지도)

**① 사전 확인 (launch 전)**

```bash
pgrep -af "ros2 launch a1_bringup"        # 비어 있어야 함. 떠 있으면 끄기
nc -zv -w2 192.168.53.100 5017            # succeeded 가 나와야 함
ls $MAP_DIR 2>/dev/null                   # 비어 있어야 함 (있으면 덮어씀 → 이름 바꾸기)
```

**② mapping 시작**

```bash
ros2 launch a1_bringup mapping.launch.py map_output_path:=$MAP_DIR/map
```

켜는 순간부터 mapping이 시작된다. 출발하기 전에 ③을 끝낸다.

**③ 출발 전 GNSS 확인 (다른 터미널)**

```bash
ros2 topic hz /a1_gnss/navsat                                              # 약 100 Hz
ros2 topic echo /gsof_client/gsof/ins_solution_49 --once --field status    # gnss 4 (RTK fixed)
ros2 topic echo /gsof_client/gsof/ins_solution_rms_50 --once               # GSOF #50 들어오는지
ros2 topic echo /a1_gnss/odom --field twist.twist.linear                   # 직진 시 x ≈ +속도, y ≈ 0
```

MOLA는 각 키프레임에 1초 이내의 GNSS 측정을 붙여 `.simplemap`에 저장하고,
georeferencing은 이 측정을 쓴다. 시작한 뒤에 GNSS 문제를 발견했다면 Ctrl-C로
끄고 다시 시작한다.

**④ 주행**: 회전이 섞인 트랙을 가능한 한 RTK fixed 상태로 달린다.

**⑤ 저장 (차를 세우고 다른 터미널에서)**

```bash
ros2 service call /map_save mola_msgs/srv/MapSave "{map_path: '$MAP_DIR/map'}"
```

응답이 `success: true`인지 확인한 다음에 launch를 Ctrl-C로 끈다.

**⑥ 전체 지도 다시 만들기 (필수)**

⑤의 `map.mm`은 MOLA의 로컬 지도라서 멈춘 위치 주변 약 100~150 m만 담긴다
(로컬 지도는 이보다 먼 키프레임을 지운다). 트랙 전체는 `map.simplemap`에만
있으므로 여기서 전체 지도를 만든다.

```bash
cd $MAP_DIR
mv map.mm map_local.mm
sm2mm -i map.simplemap -o map.mm \
  -p $WS/src/external/mp2p_icp/mp2p_icp_core/demos/sm2mm_pointcloud_voxelize_keyframe_map.yaml \
  -l libmola_metric_maps.so
mm-info map.mm                            # 범위가 트랙 전체인지 확인 (mm-viewer map.mm 으로도 확인)
```

**⑦ GNSS 확인과 georeferencing (⑥ 다음에)**

```bash
sm-cli info map.simplemap                 # gps 관측이 있는지 확인
mola-sm-georeferencing -i map.simplemap --write-into map.mm -o map.georef
```

에러 없이 끝나면 성공이며 맞춘 결과의 오차(RMSE)가 출력된다. ⑥보다 먼저 하면
georef 정보가 로컬 지도에 들어간다. 제어기 출력은 `map` 좌표라 georef 여부와
상관없이 같다.

**지도 파일**

| 파일 | 생기는 시점 | 내용 |
| --- | --- | --- |
| `map.simplemap` | ⑤ | 키프레임 포즈 + LiDAR 스캔 + GNSS 측정. 전체 지도와 georef의 입력 |
| `map_local.mm` | ⑤ → ⑥에서 이름 변경 | 멈춘 위치 주변만 담긴 로컬 지도 (보관용) |
| `map.mm` | ⑥ | 위치 추정에 쓰는 트랙 전체 지도. ⑦ 후에는 georef 정보 포함 |
| `map.georef` | ⑦ | map ↔ 위경도(ENU/UTM) 변환 |

`map_output_path`를 빼면 기본값 `<workspace>/maps/track/map`에 덮어쓴다.
`maps/`는 git에서 제외되므로 지도는 따로 백업한다.

### 위치 추정

`MAP_DIR`은 실제로 만들어진 폴더 이름으로 지정한다(`ls $WS/maps/`).

```bash
# 터미널 1
pgrep -af "ros2 launch a1_bringup"        # 비어 있어야 함
nc -zv -w2 192.168.53.100 5017            # succeeded
ros2 launch a1_bringup localization.launch.py map:=$MAP_DIR/map

# 터미널 2: 녹화 시작 (초기 포즈를 찍기 전에)
ros2 bag record -s mcap -o ~/rosbags/a1_loc_$(date +%Y%m%d_%H%M%S) \
  /localization/odom /localization/accel \
  /lidar_odometry/pose /lidar_odometry/pose_quality /diagnostics /initialpose \
  /a1_gnss/odom /a1_gnss/accel /a1_gnss/navsat \
  /gsof_client/odom /gsof_client/navsat \
  /gsof_client/gsof/ins_solution_49 /gsof_client/gsof/ins_solution_rms_50 \
  /tf /tf_static /lidar_packets_loss

# 터미널 3: GNSS 확인 → RViz 2D Pose Estimate → 활성화 → 출력 확인
ros2 topic hz /a1_gnss/odom               # 약 100 Hz (0이면 여기서 멈추고 확인)
ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
  "{parameters: \"mola::LidarOdometry:lidar_odom:\\n  active: true\\n\"}"
ros2 topic echo /lidar_odometry/pose_quality   # 0.5 이상
ros2 topic hz /localization/odom          # 약 100 Hz면 완료
```

MOLA는 꺼진 상태로 시작하므로 RViz의 **2D Pose Estimate**로 차의 위치와 방향을
찍고(초기 yaw 오차 5° 이내) 활성화해야 `/localization/odom`이 나온다. 끌 때는
녹화 → launch 순서이며, `ros2 bag info`로 `/localization/odom` 개수가 녹화
시간(초) × 100 정도인지 확인한다. 포인트 클라우드까지 녹화하려면 녹화 명령 끝에
`/lidar_points`를 추가한다(분당 약 2 GB).

두 launch 모두 `a1_localization` `mola.launch.py`의 다른 인자(`rviz:=false`,
`start_lidar_driver:=false` 등)를 뒤에 붙일 수 있다.

## LiDAR 드라이버만 실행

LiDAR와 GNSS 모두 현재는 host 수신 시각을 사용한다(PTP lock 확인 전).

```zsh
ros2 launch a1_lidar_bringup pandarxt.launch.py timestamp_type:=1
```

GNSS Grandmaster와 LiDAR PTP lock을 확인한 뒤에만 LiDAR timestamp를 사용한다.

```zsh
ros2 launch a1_lidar_bringup pandarxt.launch.py timestamp_type:=0
```

`--symlink-install`에서는 runtime YAML의 Firetime 경로가 source 디렉터리를 가리킬 수 있으며 이는 정상이다.

Firetime correction은 LiDAR 채널별 발광 시각 오차를 보정하고, PTP는 LiDAR
시계를 Grandmaster와 동기화한다. Motion deskew는 한 scan 동안 차량이
움직인 영향을 LiDAR registration twist 또는 별도 motion source로 보상하는
처리다. 어느 하나가 나머지를 대신하지 않는다.

## 문서

- [Architecture](docs/architecture.md)
- [Controller interface](docs/controller_interface.md)
- [MOLA-LO installation](docs/setup/mola_installation.md)
- [Applanix GNSS/INS integration (REP-105)](docs/setup/gnss_integration.md)
- [PandarXT setup reference](docs/setup/pandarxt_setup.md)
- [PandarXT to MOLA-LO interface](docs/setup/pandarxt_mola_interface.md)
- [Mapping](docs/operation/mola_mapping.md)
- [Map-based localization](docs/operation/mola_localization.md)

## Read-only 검증

다른 터미널에서 ROS 2와 workspace를 source한 다음 실제 인터페이스명을 전달한다.

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
./tools/analysis/verify_lidar.sh eno1
```

검증 스크립트는 네트워크 설정이나 ROS daemon 상태를 변경하지 않으며, discovery와 메시지 수신을 timeout 및 재시도로 제한한다. 핵심 성공 조건은 explicit type으로 실제 PointCloud2 메시지를 받는 것이다. header, width `128000`, point step `26`, field schema와 packet loss를 함께 확인한다. `ros2 topic echo`의 `A message was lost` 경고 자체는 센서 UDP 손실로 판정하지 않는다. 대용량 PointCloud2에서는 `ros2 topic hz`의 CLI subscriber가 실제 처리율을 낮게 표시할 수 있으므로 이 값만으로 센서 손실을 판단하지 않는다. `ros2 param get` 응답 여부도 성공 기준이 아니다.

### 간헐적인 `Unknown topic` troubleshooting

드라이버가 실행 중인데 수동 `ros2 topic info` 또는 자동 type discovery가 간헐적으로 `Unknown topic`을 반환할 때만 다음 명령으로 로컬 ROS daemon을 재시작한 뒤 다시 확인한다. 이는 troubleshooting 절차이며 `verify_lidar.sh`는 daemon을 재시작하지 않는다.

```zsh
ros2 daemon stop
ros2 daemon start
```

## 저장소 구조

```text
.
├── docs/
│   ├── architecture.md              # 데이터 흐름, 좌표계, 노드/토픽 요약
│   ├── controller_interface.md      # 제어기가 받는 /localization/* 필드 정의
│   ├── setup/
│   │   ├── gnss_integration.md      # Applanix 드라이버, extrinsics, 시각, REP-105, merger
│   │   ├── mola_installation.md     # MOLA source/dependency/build 기준
│   │   ├── pandarxt_mola_interface.md # PointCloud2/deskew/TF contract
│   │   └── pandarxt_setup.md        # LiDAR/host/PTP 설정 기준
│   └── operation/
│       ├── mola_localization.md     # map load, initial pose, output interface
│       └── mola_mapping.md          # mapping 및 map 저장 workflow
├── tools/
│   ├── host/
│   │   ├── setup_lidar_host.sh      # 호스트 NetworkManager/sysctl 설정
│   │   ├── 90-hesai-lidar.conf      # 호스트 UDP receive-buffer 설정
│   │   └── ptp4l-slave.conf         # TM2000A PTP slave 설정
│   ├── recording/
│   │   ├── record_baseline_bag.sh   # 오프라인 baseline용 raw topic 녹화
│   │   └── replay_baseline.sh       # baseline bag을 MOLA mapping/localization에 재생
│   └── analysis/
│       ├── check_pandarxt_pointcloud.py # field/timestamp live diagnostic
│       └── verify_lidar.sh          # read-only LiDAR/ROS 검증
├── maps/                            # MOLA map 출력 (git 제외)
└── src/
    ├── a1/                          # 프로젝트 코드
    │   ├── a1_bringup/              # 최상위 launch (mapping, localization)
    │   ├── a1_gnss_bringup/         # Applanix launch/config, extrinsics, odom relay, merger
    │   ├── a1_lidar_bringup/        # PandarXT launch/config/calibration
    │   └── a1_localization/         # MOLA launch/config
    └── external/                    # 외부 코드 (Git submodule, 직접 수정하지 않음)
        ├── HesaiLidar_ROS_2.0/
        ├── mola_lidar_odometry/     # pinned 3.0.0
        ├── mola_metric_maps -> vendor/mola/mola_metric_maps
        ├── mp2p_icp/
        ├── trimble_driver_ros/      # humble branch
        └── vendor/mola/             # MOLA 3.1.1 (mola_metric_maps만 빌드)
```
