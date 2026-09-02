# PandarXT setup reference

이 문서는 LiDAR 장치 내부 설정과 LiDAR UDP 데이터를 수신하는 호스트 PC 설정을 분리해 기록한다. 장치를 다른 노트북이나 실차 컴퓨터로 옮길 때 두 영역을 각각 확인해야 한다.

## LiDAR 내부 설정

다음 값은 PandarXT 웹 설정 화면 등에서 관리하는 **LiDAR 장치 내부 설정**이다. 호스트 NetworkManager 또는 Linux 커널 설정과 혼동하지 않는다.

| 항목 | 값 |
| --- | --- |
| Model | PandarXT |
| Control IP | `192.168.1.201` |
| Mask | `255.255.255.0` |
| Gateway | `192.168.1.1` |
| VLAN | OFF |
| Destination IP | `255.255.255.255` |
| Destination port | `2368` |
| Spin rate | `600 rpm` = `10 Hz` |
| Return mode | Last Return + Strongest Return |
| Sync Angle | OFF |
| Trigger method | Angle Based |
| Clock source | PTP |
| PTP profile | 1588v2 |
| PTP transport | UDP/IP |
| PTP domain | 0 |
| LiDAR lock offset | 1 us |
| logAnnounceInterval | 1 |
| logSyncInterval | 1 |
| logMinDelayReqInterval | 0 |
| Reflectivity Mapping | Linear |
| Interstitial Points Filtering | OFF |
| Standby Mode | In Operation |
| Azimuth FOV | all channels, 0–360 degrees |

Mapping 데이터 수집과 localization 실행에서 Return Mode를 동일하게 유지해야 포인트 구성 차이로 인한 입력 불일치를 피할 수 있다.

PandarXT 테스트에서는 `/lidar_imu` publisher만 생성되고 10초 동안 실제 IMU 메시지는 발행되지 않았다. PandarXT가 유효한 IMU 데이터를 제공한다고 가정하지 않으며 `send_imu_ros`는 비활성화한다. 향후 motion deskew와 localization에는 외부 IMU를 사용하고 `/imu/data_raw` 같은 별도 topic 이름으로 연결할 예정이다.

## 호스트 PC 설정

호스트 설정은 LiDAR 내부가 아니라 **LiDAR UDP 데이터를 수신하는 각 PC**에 적용한다. 저장소 루트에서 다음을 실행한다. `eno1`은 예시이므로 실제 Ethernet 인터페이스명으로 바꾼다.

```bash
sudo ./scripts/setup_lidar_host.sh eno1
```

스크립트의 기본값은 다음과 같다.

- 호스트 주소: `192.168.1.100/24`
- NetworkManager connection: `a1-lidar`
- IPv4: manual
- gateway와 DNS: 비움
- `ipv4.never-default`: `yes`
- 해당 LiDAR 전용 연결의 IPv6: disabled
- `net.core.rmem_max`: `33554432`

connection 이름은 필요한 경우 `LIDAR_CONNECTION_NAME` 환경 변수로 바꿀 수 있다. 스크립트는 대상 연결을 다시 올리므로 Ethernet 연결이 잠시 끊길 수 있다. 특히 현재 SSH 세션이 그 인터페이스를 사용한다면 세션이 종료될 수 있다.

`system/90-hesai-lidar.conf`는 각 수신 호스트의 `/etc/sysctl.d/90-hesai-lidar.conf`에 설치된다. Linux는 `SO_RCVBUF`에 설정한 값을 내부 bookkeeping 공간까지 포함해 두 배로 표시할 수 있다. 따라서 드라이버가 `67108864 bytes`를 표시하는 것은 `33554432` 요청값에 대한 정상 동작이며, LiDAR 내부 메모리 설정을 뜻하지 않는다.

Destination IP가 broadcast이므로 PC를 한 번에 한 대만 연결한다면 각 PC에서 기본 `.100` 주소를 재사용할 수 있다. 여러 PC를 같은 센서 네트워크에 동시에 연결할 때는 주소 충돌을 피하도록 각 호스트에 서로 다른 `192.168.1.X/24` 주소를 설정해야 한다.

```bash
sudo ./scripts/setup_lidar_host.sh enp3s0 192.168.1.101/24
```

## 시간 동기화

LiDAR의 Clock source가 PTP여도 현재 Grandmaster가 없으므로 PTP 상태가 Free Run인 것은 정상이다. 지금은 host receive time을 사용한다.

```bash
ros2 launch a1_lidar_bringup pandarxt.launch.py timestamp_type:=1
```

GNSS Grandmaster가 준비되고 LiDAR의 PTP lock이 실제로 확인된 뒤에만 LiDAR timestamp를 사용한다.

```bash
ros2 launch a1_lidar_bringup pandarxt.launch.py timestamp_type:=0
```

## Runtime 설정 파일

launch 파일은 package share의 `config/pandarxt.yaml.in`을 읽어 Firetime 절대경로와 timestamp type을 치환한다. 결과는 `${ROS_HOME}/a1_localization/pandarxt_runtime.yaml`에 생성되며, `ROS_HOME`이 비어 있으면 `~/.ros/a1_localization/pandarxt_runtime.yaml`을 사용한다.

`--symlink-install`로 빌드한 환경에서는 package share의 calibration 경로가 source 디렉터리를 가리킬 수 있다. 따라서 runtime YAML의 Firetime 경로가 source 아래를 가리키는 것은 정상이다.

## Firetime, PTP, motion deskew의 구분

- **Firetime correction**은 레이저 채널별 발광 시각 차이를 보정하는 LiDAR calibration이다.
- **PTP 동기화**는 LiDAR clock을 GNSS Grandmaster 등 기준 시계와 맞추는 작업이다.
- **Motion deskew**는 한 scan을 수집하는 동안 차량이 이동한 영향을 외부 IMU 또는 odometry로 보상하는 별도 알고리즘이다.

Firetime이나 PTP가 motion deskew를 자동으로 수행하지 않으며, deskew에는 시간 정렬된 외부 motion 입력이 필요하다.

## 검증과 해석 주의사항

ROS 2 Humble과 workspace를 source하고 드라이버를 실행한 상태에서 read-only 검증을 수행한다.

```bash
./scripts/verify_lidar.sh eno1
```

스크립트는 인터페이스, receive buffer, ping, `/lidar_points`, PointCloud2 header/width/point step/fields, `/lidar_packets_loss`, 커널 UDP 오류 카운터를 확인한다. Discovery와 실제 메시지 수신에는 timeout과 재시도를 적용하며 ROS daemon은 변경하지 않는다.

- `/lidar_points` echo에는 `sensor_msgs/msg/PointCloud2` explicit type, `--no-daemon`, best-effort reliability와 depth 1을 사용한다.
- 실제 PointCloud2 메시지 수신을 핵심 성공 조건으로 사용한다. Topic info 조회가 실패해도 메시지를 받았다면 publisher 검사는 경고로 남는다.
- 검증 schema는 frame `hesai_lidar`, width `128000`, point step `26`, 그리고 `x`, `y`, `z`, `intensity`, `ring`, `timestamp` fields다.
- Topic info가 정상 조회되면 `/lidar_points` publisher는 정확히 1개여야 한다.
- CLI의 `A message was lost` 경고 자체를 LiDAR UDP packet loss로 판정하지 않는다. 센서 손실은 `/lidar_packets_loss`와 커널 UDP 카운터로 별도 확인한다.
- 대용량 PointCloud2에서는 `ros2 topic hz`의 CLI subscriber가 실제 처리율보다 낮은 값을 표시할 수 있으므로 이것만으로 센서 손실을 판정하지 않는다.
- `ros2 param get` 응답 여부를 성공 기준으로 사용하지 않는다.
- `UdpInErrors`와 `UdpRcvbufErrors`는 호스트 전체의 누적값이다. 한 번의 실행을 진단하려면 실행 전후 증가량을 비교한다.
- `/lidar_imu` publisher가 보이더라도 실제 IMU 메시지가 발행되는 것은 아니다. PandarXT IMU를 localization 입력으로 사용하지 않는다.

## `Unknown topic` troubleshooting

드라이버가 실행 중인데 수동 `ros2 topic info`나 자동 type discovery가 간헐적으로 `Unknown topic`을 반환하는 경우에만 로컬 daemon을 재시작하고 다시 확인한다.

```bash
ros2 daemon stop
ros2 daemon start
```

이 절차는 수동 troubleshooting 전용이다. `verify_lidar.sh`는 daemon을 임의로 재시작하지 않고 `--no-daemon`과 재시도로 직접 DDS discovery를 수행한다.
