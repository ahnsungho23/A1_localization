# A1 localization workspace

ROS 2 Humble 기반 A1 실차 localization workspace다. 현재 범위는 Hesai PandarXT bringup이며, 향후 3D LiDAR localization, GNSS localization 및 센서 fusion을 추가할 예정이다.

## 지원 환경

- Ubuntu 22.04
- ROS 2 Humble

## 현재 상태

실차에서 다음 PandarXT bringup 항목까지 검증됐다.

- `a1_lidar_bringup` 빌드 성공
- 이 workspace의 Hesai 드라이버 실행 성공
- 실제 ROS 노드 이름: `/hesai_ros_driver_node`
- `/lidar_points`: `sensor_msgs/msg/PointCloud2`, publisher 1개, Reliability `RELIABLE`
- explicit type과 best-effort subscriber QoS로 실제 PointCloud2 메시지 수신 성공
- PointCloud2 frame: `hesai_lidar`
- PointCloud2 width: `128000`
- PointCloud2 point step: `26` bytes
- PointCloud2 fields: `x`, `y`, `z`, `intensity`, `ring`, `timestamp`
- `/lidar_packets_loss`: 손실 0
- `timestamp_type:=1`에서 PointCloud2 header 시간이 host 시간과 일치
- launch에서 runtime YAML과 Firetime 경로 자동 생성
- PandarXT 테스트에서 `/lidar_imu` publisher만 생성되고 10초 동안 실제 IMU 메시지는 발행되지 않음

현재는 **LiDAR bringup까지만 검증됐으며 localization 알고리즘은 아직 구현 전**이다.

PandarXT의 `/lidar_imu`를 localization 입력으로 사용하지 않는다. 향후 motion deskew와 localization에는 외부 IMU가 필요하며, 외부 장치는 `/imu/data_raw` 같은 별도 topic 이름을 사용할 예정이다.

## 저장소 받기

Hesai 드라이버와 내부 SDK가 submodule이므로 반드시 recursive clone을 사용한다. `<repository-url>`은 GitHub 원격 저장소가 생성된 뒤 실제 URL로 교체해야 하는 placeholder다.

```bash
git clone --recurse-submodules <repository-url> a1_localization_ws
cd a1_localization_ws
```

이미 일반 clone을 했다면 다음 명령으로 submodule을 초기화한다.

```bash
git submodule update --init --recursive
```

## 호스트 네트워크 설정

다음 명령은 LiDAR UDP를 수신하는 **각 호스트 PC**에서 실행한다. `eno1`은 예시이므로 실제 Ethernet 인터페이스명으로 바꾼다. 대상 연결을 올리는 동안 Ethernet이 잠시 끊길 수 있으며, 해당 인터페이스를 사용하는 SSH 세션도 종료될 수 있다.

```bash
sudo ./scripts/setup_lidar_host.sh eno1
```

기본 호스트 주소는 `192.168.1.100/24`다. 여러 PC를 센서 네트워크에 동시에 연결한다면 각 호스트에 서로 다른 주소를 지정한다.

```bash
sudo ./scripts/setup_lidar_host.sh enp3s0 192.168.1.101/24
```

LiDAR 내부 설정, PTP 조건 및 receive-buffer 해석은 [PandarXT setup reference](docs/pandarxt_setup.md)를 참고한다.

## 의존성 설치와 빌드

```zsh
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.zsh
```

## 실행

현재는 PTP Grandmaster가 없으므로 host receive time을 사용한다.

```zsh
ros2 launch a1_lidar_bringup pandarxt.launch.py timestamp_type:=1
```

GNSS Grandmaster와 LiDAR PTP lock을 확인한 뒤에만 LiDAR timestamp를 사용한다.

```zsh
ros2 launch a1_lidar_bringup pandarxt.launch.py timestamp_type:=0
```

`--symlink-install`에서는 runtime YAML의 Firetime 경로가 source 디렉터리를 가리킬 수 있으며 이는 정상이다.

Firetime correction은 LiDAR 채널별 발광 시각 오차를 보정하고, PTP는 LiDAR 시계를 Grandmaster와 동기화한다. Motion deskew는 한 scan 동안 차량이 움직인 영향을 외부 IMU/odometry로 보상하는 별도 처리다. 어느 하나가 나머지를 대신하지 않는다.

## Read-only 검증

다른 터미널에서 ROS 2와 workspace를 source한 다음 실제 인터페이스명을 전달한다.

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
./scripts/verify_lidar.sh eno1
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
│   └── pandarxt_setup.md        # LiDAR/host/PTP 설정 기준
├── scripts/
│   ├── setup_lidar_host.sh      # 호스트 NetworkManager/sysctl 설정
│   └── verify_lidar.sh          # read-only LiDAR/ROS 검증
├── system/
│   └── 90-hesai-lidar.conf      # 호스트 UDP receive-buffer 설정
└── src/
    ├── HesaiLidar_ROS_2.0/      # Git submodule
    └── a1_lidar_bringup/        # PandarXT launch/config/calibration
```
