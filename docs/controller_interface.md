# 제어기 인터페이스

`localization_odom_merger`가 GNSS/INS 샘플마다(약 100 Hz) 두 토픽을 같은
`header.stamp`로 발행한다. 첫 LO 포즈가 들어오기 전에는 아무것도 발행하지 않는다.

## `/localization/odom` (`nav_msgs/msg/Odometry`)

`header.frame_id = map`, `child_frame_id = base_link`
(`base_link`: 뒷바퀴 축 중심의 지면 위 점, x 전방, y 좌측, z 위)

| 필드 | 단위 | 내용 |
| --- | --- | --- |
| `pose.pose.position.x/y/z` | m | `map` 기준 `base_link` 위치 |
| `pose.pose.orientation` | quaternion | `map` 기준 `base_link` 자세. LO 보정 시점에는 LO 자세, 그 사이는 INS 회전량으로 이음 |
| `pose.covariance` | | 마지막으로 채택된 LO 포즈의 covariance |
| `twist.twist.linear.x/y` | m/s | INS 속도, `base_link` 축(x 전방, y 좌측) |
| `twist.twist.angular.z` | rad/s | INS yaw rate |
| **`twist.twist.angular.x`** | **deg** | **각속도가 아님. `map` x축 기준 차량 heading, 반시계 +, (-180, 180]** |
| 나머지 twist | | 0 |
| `twist.covariance` | | 고정값: σ_v = 0.05 m/s, σ_ωz = 0.02 rad/s, 나머지 1e6 |

heading은 같은 메시지의 quaternion에서 계산한 값이다.
`atan2(2(wz + xy), 1 − 2(y² + z²))`. ±180°에서 값이 넘어가므로 각도 차이를
계산할 때 wrap 처리가 필요하다.

## `/localization/accel` (`geometry_msgs/msg/AccelWithCovarianceStamped`)

`header.frame_id = base_link`

| 필드 | 단위 | 내용 |
| --- | --- | --- |
| `accel.accel.linear.x/y` | m/s² | INS 가속도(GSOF #49)를 `base_link` 축으로 돌리고 두 점 정리(yaw 성분)로 `base_link` 위치의 가속도로 옮긴 값의 최근 0.4 s 평균. 약 0.2 s 늦음 |
| `accel.covariance` | | 고정값: σ_a = 0.3 m/s². 평균할 샘플이 없으면 `[0] = -1` |

## 정차 hold

연속된 LO 포즈 5개(0.5 s)가 모두 1 cm, 0.1° 미만으로 움직이면 정차로 본다.
정차 중에는 다음과 같이 발행한다.

- 위치·자세: 최신 LO 포즈. GNSS 움직임은 무시한다.
- `twist.linear.x/y`, `twist.angular.z`: 0. `twist.angular.x`(heading)는 그대로 발행.
- `/localization/accel`: 그대로 INS 값.

LO가 정차 시작 위치에서 5 cm 넘게 움직이거나, INS 속도가 0.15 m/s를 넘고
마지막 LO 스캔도 1.5 cm 넘게 움직이면 해제한다. 출발 직후 0.1~0.3 s 동안은
속도가 0으로 나갈 수 있다. merger 파라미터 `stationary_hold:=false`로 끈다.
