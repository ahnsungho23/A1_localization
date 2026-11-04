# Applanix GNSS/INS integration (REP-105)

## Architecture

```text
map ──[MOLA bridge]──▶ odom ──[gnss_odom_relay]──▶ base_link ──[static, measured]──▶ ins
                                                            └──[static, measured]──▶ hesai_lidar
```

- `odom -> base_link`: Applanix INS pose (`/gsof_client/odom`, INS output
  point `ins`) moved to `base_link` with the measured lever arm.
- `map -> odom`: MOLA-LO correction, composed by `mola_bridge_ros2` from the
  LiDAR `map -> base_link` result and `base_link -> odom` looked up at the
  scan timestamp (`publish_localization_following_rep105=true`).
- `localization_odom_merger` combines the INS motion and the LO pose into the
  100 Hz controller output `/localization/odom` and `/localization/accel`
  (see [below](#controller-output-localizationodom-localizationaccel)).
- GNSS fixes are recorded into the mapping `.simplemap` for offline
  georeferencing. MOLA-LO does not use them for pose estimation.

## Driver

`src/external/trimble_driver_ros` is the `trimble-oss/trimble_driver_ros`
submodule, pinned to the `humble` branch. Its parameters are not edited in
place; `a1_gnss_bringup/config/gsof_client.yaml` overrides them:

| Parameter | Value | Reason |
| --- | --- | --- |
| `ip`, `port` | `192.168.53.100`, `5017` | Applanix GSOF TCP output; `ip:=` on the launch overrides it |
| `publish_tf` | `false` | the relay owns `odom -> base_link`; a driver TF would give `ins` two parents |
| `publish_rep103` | `true` | ENU position, FLU orientation |
| `child_frame` | `ins` | must match `base_link_to_ins.frame_id` |
| `time_source` | `now` | host receive time, the same clock the PandarXT driver uses (`timestamp_type:=1`) |

On the Applanix web interface (`IO Configuration > Port Summary`), enable GSOF
**#49** (INS Full Navigation Info) and **#50** (INS RMS Info) on the TCP port.
Without #50 the NavSatFix has no covariance and `mola-sm-georeferencing`
discards every GNSS sample.

Do not move the Applanix output point to `base_link` on the device: the lever
arm is applied by the static `base_link -> ins` transform, and applying it in
both places doubles it.

The Applanix vehicle frame (mounting angles) is set on the device so that the
INS x axis points forward. Velocity and acceleration are expressed in that
frame, so if it is rotated, `vx/vy` and `ax/ay` are rotated with it. Check on a
straight drive that `/a1_gnss/odom` shows `vx` ≈ speed and `vy` ≈ 0.

## Vehicle extrinsics

`a1_gnss_bringup/config/vehicle_extrinsics.yaml` holds the measured mounting
poses relative to `base_link` (REP-103 axes, meters, degrees). The `base_link`
origin is the center of the rear axle on the ground (z = 0 at ground level):

| Transform | x, y, z | roll, pitch, yaw |
| --- | --- | --- |
| `base_link -> ins` | 0.84, 0.0, 0.50 | 0, 0, 0 |
| `base_link -> hesai_lidar` | 1.16, 0, 1.20 | 0, 0, 90 |

The LiDAR is mounted rotated +90° about z. Both launch files refuse to start
while any value is `null`.

## Time

With `time_source: now` the driver stamps every message with the host clock on
receipt, and `gnss.launch.py` sets the relay's `time_conversion` to `none`.
In the 2026-10-08 bags the receive time of GNSS and the LiDAR stamps (host
receive time, `timestamp_type:=1`) agreed within 0.03 s. Applanix output and
TCP latency are not compensated.

`time_source: gps` stamps GPS time counted from 1980-01-06; the relay then
converts every stamp:

```text
unix = gps + 315964800 - gps_utc_offset_sec (18) + extra_time_offset_sec (0)
```

Switch to `gps` (and the LiDAR to `timestamp_type:=0`) only once ptp4l keeps
the host clock locked to the TM2000A
([tools/host/ptp4l-slave.conf](../../tools/host/ptp4l-slave.conf)). In the
2026-10-08 bags, recorded with `gps` and no lock, GNSS stamps were 1.3–2.3 s
behind the LiDAR stamps. The relay warns when a stamp differs from the host
clock by more than `max_clock_offset_sec` (1 s).

## Relay outputs (`gnss_odom_relay`, ~100 Hz)

| Topic | Type | Content |
| --- | --- | --- |
| `/tf` | | `odom -> base_link` |
| `/a1_gnss/odom` | `nav_msgs/Odometry` | pose `odom -> base_link`; twist in `base_link` axes: `v_b = R_bi v_i - ω_b × p_bi`, `ω_b = R_bi ω_i`; covariances passed through from the driver |
| `/a1_gnss/accel` | `geometry_msgs/AccelWithCovarianceStamped` | GSOF #49 `(acc_long, -acc_trans, -acc_down)` rotated into `base_link` axes; gravity-free, unfiltered; lever-arm terms not added; `covariance[0] = -1` (no uncertainty in #49) |
| `/a1_gnss/navsat` | `sensor_msgs/NavSatFix` | driver NavSatFix with the converted stamp; recorded by MOLA for georeferencing |

## Controller output (`/localization/odom`, `/localization/accel`)

`localization_odom_merger` publishes one message on each topic per
`/a1_gnss/odom` sample (same stamp), once the first LO pose has arrived. Field
definitions are in [controller_interface.md](../controller_interface.md).

- **Pose (`map`)**: `C · G(t)`, where `G` is the GNSS pose and
  `C = L(t_s) · G(t_s)⁻¹` is updated for every LO pose `L` (scan stamp `t_s`,
  GNSS interpolated at `t_s`). Between LO poses the output moves with the INS.
  A new `C` is blended in over 0.1 s.
- **Rejection**: an LO update is ignored when its ICP quality is below 0.5 or
  it would move the output more than 0.5 m / 5°. Five consecutive mutually
  consistent rejected updates are applied as a step (resync).
- **Stationary hold**: when five consecutive LO poses moved less than 1 cm and
  0.1°, the output follows the LO pose directly, GNSS motion is ignored and the
  twist is zero. It ends when LO moved more than 5 cm, or the GNSS speed exceeds
  0.15 m/s while the last LO step exceeds 1.5 cm; `C` is then re-anchored.
- **Twist**: `/a1_gnss/odom` `linear.x/y`, `angular.z` (`base_link` axes);
  `angular.x` carries the heading in degrees from the `map` x axis.
- **Accel**: mean of `/a1_gnss/accel` over the last 0.4 s.

## Launch

Vehicle mapping and localization (LiDAR, GNSS, vehicle TF, MOLA, merger):

```zsh
ros2 launch a1_bringup mapping.launch.py
ros2 launch a1_bringup localization.launch.py
```

Both include `a1_localization` `mola.launch.py`, which always runs MOLA in
REP-105 mode with `gnss_topic_name` set to `/a1_gnss/navsat`, includes
`gnss.launch.py` (disable with `start_gnss_driver:=false` if it runs
separately) and starts the merger. There is no LiDAR-only mode; the LiDAR pose
always comes from `vehicle_extrinsics.yaml` through TF.

GNSS stack alone (driver, static TFs, relay):

```zsh
ros2 launch a1_gnss_bringup gnss.launch.py
```

Use `start_driver:=false` to replay recorded `/gsof_client/*` topics through
the relay.

## Georeferencing (after mapping)

```zsh
mola-sm-georeferencing -i map.simplemap --write-into map.mm -o map.georef
```

The tool needs at least three non-collinear keyframes with GNSS. Keyframes get
the closest fix within 1 s, so a high GSOF #49 rate reduces the
time-mismatch error. GNSS samples are weighted by their NavSatFix covariance
(floor 0.2 m).

## Observed behavior (2026-10-08 bags)

- The INS alignment status stayed `ALIGNED` (3) for the whole drive, also with
  RTK fixed; it never reported `FULL_NAV` (4). Use the GNSS status and the
  GSOF #50 position RMS, not the alignment status, to judge RTK quality.
- Right after start-up or RTK re-acquisition the status can read RTK fixed
  while the position RMS is still 0.1–2.9 m.
- Without RTK (DGPS/SPS) the INS position drifted up to ~10 m from the LO
  trajectory. During DGPS standstills it reported 0.6–0.7 m/s while the vehicle
  was stopped and then jumped ~3.5 m; the stationary hold removes this from the
  output (6.9 m → 0.14 m in replay).
- Replay of the merger against the LO trajectory: 7–9 cm rms position error
  while driving with RTK fixed, 4–6 cm rms while driving without RTK.
