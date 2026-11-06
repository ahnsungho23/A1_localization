# 제어기 인터페이스

`localization_odom_merger`가 GNSS/INS 샘플마다(약 100 Hz) 두 토픽을 같은
`header.stamp`로 발행한다. 첫 LO 포즈가 들어오기 전에는 아무것도 발행하지 않는다.

## `/localization/odom` (`nav_msgs/msg/Odometry`)

`header.frame_id = map`, `child_frame_id = base_link`
(`base_link`: 뒷바퀴 축 중심의 지면 위 점, x 전방, y 좌측, z 위)

| 필드 | 단위 | 내용 |
| --- | --- | --- |
| `pose.pose.position.x/y/z` | m | `map` 기준 `base_link` 위치 |
| `pose.pose.orientation` | quaternion | `map` 기준 `base_link` 자세. INS 자세에 LO 보정을 조금씩(아래 "LO 보정") 반영한 값 |
| `pose.covariance` | | 모두 0 (제어기는 쓰지 않음. `publish_covariance:=true`면 마지막 LO 포즈의 covariance) |
| `twist.twist.linear.x/y` | m/s | INS 속도, `base_link` 축(x 전방, y 좌측) |
| `twist.twist.angular.z` | rad/s | INS yaw rate |
| **`twist.twist.angular.x`** | **deg** | **각속도가 아님. `map` x축 기준 차량 heading, 반시계 +, (-180, 180]** |
| 나머지 twist | | 0 |
| `twist.covariance` | | 모두 0 |

heading은 같은 메시지의 quaternion에서 계산한 값이다.
`atan2(2(wz + xy), 1 − 2(y² + z²))`. ±180°에서 값이 넘어가므로 각도 차이를
계산할 때 wrap 처리가 필요하다.

## `/localization/accel` (`geometry_msgs/msg/AccelWithCovarianceStamped`)

`header.frame_id = base_link`

| 필드 | 단위 | 내용 |
| --- | --- | --- |
| `accel.accel.linear.x` | m/s² | INS 가속도(GSOF #49, 두 점 정리로 `base_link`로 옮긴 값)를 2차 Butterworth 3 Hz 저역통과한 값. 약 0.08 s 늦음 |
| `accel.accel.linear.y` | m/s² | INS 속도로 계산한 횡가속도: vx·ωz(각각 최근 0.1 s 평균) + dvy/dt(1.5 Hz 저역통과). 원시 횡가속도보다 진동 잡음이 작음 |
| `accel.covariance` | | 모두 0. 최근 0.1 s 안에 INS 가속도가 없으면 가속도도 0 |

## LO 보정

LO 포즈가 들어올 때마다 LO와 현재 출력의 차이(차량 좌표) 중 30%만 반영한다.
한 번에 움직이는 양은 위치 max(5 cm, 직전 보정 이후 이동거리의 6%), yaw 0.3°로
제한한다. 그래서 LO 한 스캔이 미끄러져도 출력은 몇 cm만 움직이고, 출력이 계단처럼
뛰지 않는다. RTK fixed가 아니거나 INS가 aligned가 아니면 한 번에 0.3 m, 2°까지
허용한다.

## GNSS 점프와 LO 기준 모드

GNSS 위치가 직전 샘플의 INS 속도·yaw rate로 설명되지 않게 0.5 m 또는 3° 넘게
움직이면(RTK 손실·복귀, INS 재정렬), 보정값이 그 점프를 바로 흡수한다. 출력은
점프 직전 위치에서 INS가 예측한 대로 이어지고, LO가 위 제한대로 맞춘다.

INS가 GPS_ONLY/COARSE_LEVELING이거나 GSOF #50 heading RMS가 2°를 넘으면 GNSS heading을
쓰지 않는다(GPS_ONLY에서는 heading이 90°로 고정되고 yaw rate가 0, 5 Hz).

- 위치: LO 포즈마다 LO 쪽으로 옮기되(한 번에 최대 0.3 m), 그 사이는 GNSS 이동량으로 잇는다.
- heading: LO heading. yaw rate(`twist.angular.z`): 연속된 LO heading의 변화율.
- 속도: GNSS 속도를 LO heading 기준 차량 축으로 돌린 값.
- 가속도: x는 GNSS 속력의 미분, y는 속력 × LO yaw rate.

## 정차 hold

다음 조건이 0.2 s 이어지면 정차로 본다.

- `/control/status/wheel_info`의 좌우 바퀴속도가 모두 0, 메시지 stamp가 0.2 s 이내,
  INS 속도 0.1 m/s 미만
- 바퀴속도가 없거나 오래된 경우: RTK fixed이고 INS aligned일 때만 INS 속도 0.05 m/s 미만

정차 중에는 다음과 같이 발행한다.

- 위치·자세: 정차 직전 출력에 고정. GNSS 움직임과 LO 보정은 무시한다.
- `twist.linear.x/y`, `twist.angular.z`: 0. `twist.angular.x`(heading)는 그대로 발행.
- `/localization/accel`: 그대로 계산한 값.

조건이 깨지면 바로 해제하고, 고정했던 위치에서 이어서 발행한다.
merger 파라미터 `stationary_hold:=false`로 끈다.
