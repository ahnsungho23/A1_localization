# A1 localization 구조

## 데이터 흐름

```text
PandarXT ──▶ hesai_ros_driver ──/lidar_points (10 Hz)──▶ MOLA-LO ──/lidar_odometry/pose (10 Hz, map→base_link)──┐
                                                                  └─/lidar_odometry/pose_quality─────────────────┤
                                                                                                                 ▼
Applanix ──▶ gsof_client ──/gsof_client/odom, navsat, gsof/ins_solution_49 (100 Hz)──▶ gnss_odom_relay   localization_odom_merger ──▶ 제어기
                                                                                       │  /a1_gnss/odom (odom→base_link) ──────▶   /localization/odom  (100 Hz)
                                                                                       │  /a1_gnss/accel ───────────────────────▶   /localization/accel (100 Hz)
                                                                                       └  /a1_gnss/navsat ──▶ MOLA (mapping 시 simplemap에 기록)
```

- **MOLA-LO**: 스캔을 지도에 ICP로 맞춰 `map` 기준 차량 포즈를 낸다. GNSS/IMU는
  포즈 추정에 쓰지 않는다.
- **gnss_odom_relay**: Applanix 출력(INS 장착점 `ins`)을 `base_link`로 옮기고,
  속도·가속도를 `base_link` 축으로 돌린다.
- **localization_odom_merger**: GNSS 포즈 사이의 움직임을 LO 포즈로 보정해
  100 Hz `map` 기준 출력을 만든다. 속도·가속도는 INS 값이다. LO로 정차를 판단하면
  위치는 LO를 그대로 쓰고 속도는 0으로 낸다(stationary hold). 필드 정의는
  [controller_interface.md](controller_interface.md).

## 좌표계 (REP-105, `use_gnss:=true`)

```text
map ──[MOLA bridge]──▶ odom ──[gnss_odom_relay]──▶ base_link ──[static]──▶ ins
                                                            └──[static]──▶ hesai_lidar
```

- `map → odom`: MOLA. `odom → base_link`: relay(INS).
- `base_link → ins`, `base_link → hesai_lidar`: `a1_gnss_bringup/config/vehicle_extrinsics.yaml`
  값으로 static TF 발행.
- `map`은 MOLA 지도 좌표계다. x축은 동쪽이 아니라 지도를 만들 때의 기준 방향이다.

## 시각

- PandarXT: `timestamp_type:=1`이면 호스트 수신 시각.
- Applanix: `gsof_client.yaml`의 `time_source: "now"`이면 호스트 수신 시각.
  relay는 시각 변환을 하지 않는다.
- merger는 LO 스캔 시각과 GNSS 시각을 같은 기준으로 보고 짝짓는다.

## Launch 구성

```text
a1_bringup/mapping.launch.py, localization.launch.py   (mode, use_gnss:=true, map)
└── a1_localization/mola.launch.py
    ├── a1_lidar_bringup/pandarxt.launch.py            (start_lidar_driver)
    ├── a1_gnss_bringup/gnss.launch.py                 (start_gnss_driver)
    │   ├── gsof_client (Applanix 드라이버)
    │   ├── static TF: base_link→ins, base_link→hesai_lidar
    │   └── gnss_odom_relay
    ├── localization_odom_merger                      (use_gnss일 때)
    └── mola_lidar_odometry ros2-lidar-odometry.launch.py
```

## 패키지

| 위치 | 역할 |
| --- | --- |
| `src/a1/a1_bringup` | 최상위 launch |
| `src/a1/a1_localization` | MOLA launch와 설정 |
| `src/a1/a1_gnss_bringup` | Applanix 설정, 차량 extrinsics, relay, merger |
| `src/a1/a1_lidar_bringup` | PandarXT 설정과 calibration |
| `src/external/*` | 외부 코드(submodule). 직접 수정하지 않는다 |
