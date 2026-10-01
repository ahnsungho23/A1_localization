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
- GNSS fixes are recorded into the mapping `.simplemap` for offline
  georeferencing. MOLA-LO does not use them for pose estimation.

## Driver

`src/trimble_driver_ros` is the `trimble-oss/trimble_driver_ros` submodule,
pinned to the `humble` branch. Its parameters are not edited in place;
`a1_gnss_bringup/config/gsof_client.yaml` overrides them:

| Parameter | Value | Reason |
| --- | --- | --- |
| `publish_tf` | `false` | the relay owns `odom -> base_link`; a driver TF would give `ins` two parents |
| `publish_rep103` | `true` | ENU position, FLU orientation |
| `child_frame` | `ins` | must match `base_link_to_ins.frame_id` |
| `time_source` | `gps` | GPS-epoch stamps, converted to UTC by the relay |
| `ip`, `port` | empty / `5017` | launch refuses to start without an IP |

On the Applanix web interface (`IO Configuration > Port Summary`), enable GSOF
**#49** (INS Full Navigation Info) and **#50** (INS RMS Info) on the TCP port.
Without #50 the NavSatFix has no covariance and `mola-sm-georeferencing`
discards every GNSS sample.

Do not move the Applanix output point to `base_link` on the device: the lever
arm is applied by the static `base_link -> ins` transform, and applying it in
both places doubles it.

## Vehicle extrinsics

`a1_gnss_bringup/config/vehicle_extrinsics.yaml` holds the measured
`base_link -> ins` and `base_link -> hesai_lidar` poses (REP-103 axes, meters,
degrees). Every value is `null` until measured, and both launch files refuse
to start while any value is null.

## Time

The host clock follows the TM2000A through ptp4l
([system/ptp4l-slave.conf](../system/ptp4l-slave.conf)). With
`time_source: gps` the driver stamps GPS time counted from 1980-01-06, so
`gnss_odom_relay` converts every output stamp:

```text
unix = gps + 315964800 - gps_utc_offset_sec (18) + extra_time_offset_sec (0)
```

It warns when a converted stamp differs from the host clock by more than
`max_clock_offset_sec` (1 s). Whether PandarXT PTP stamps are TAI or UTC is
not yet verified; if they are TAI, set `extra_time_offset_sec` accordingly.
Use `timestamp_type:=0` for the LiDAR once PTP lock is confirmed.

## Launch

GNSS stack alone (driver, static TFs, relay):

```zsh
ros2 launch a1_gnss_bringup gnss.launch.py ip:=<applanix-ip>
```

Relay outputs: `/tf` (`odom -> base_link`), `/a1_gnss/odom`, and
`/a1_gnss/navsat` (NavSatFix with UTC stamps). Use `start_driver:=false` to
replay recorded `/gsof_client/*` topics through the relay.

Mapping or localization with GNSS:

```zsh
ros2 launch a1_mola_localization mapping.launch.py \
  use_gnss:=true gnss_ip:=<applanix-ip> timestamp_type:=0

ros2 launch a1_mola_localization localization.launch.py \
  use_gnss:=true gnss_ip:=<applanix-ip> timestamp_type:=0 \
  map:=${HOME}/.ros/a1_localization/maps/track/map
```

`use_gnss:=true` sets MOLA's `gnss_topic_name` to `/a1_gnss/navsat`, switches
MOLA to REP-105, and includes `gnss.launch.py` (disable with
`start_gnss_driver:=false` if it runs separately). It cannot be combined with
`use_fixed_lidar_pose:=true`, because the LiDAR pose then comes from
`vehicle_extrinsics.yaml`.

## Georeferencing (after mapping)

```zsh
mola-sm-georeferencing -i map.simplemap --write-into map.mm -o map.georef
```

The tool needs at least three non-collinear keyframes with GNSS. Keyframes get
the closest fix within 1 s, so a high GSOF #49 rate reduces the
time-mismatch error.

## Status

Driver build and relay math (offline fake-message test) are verified. Live
Applanix connection, lever-arm measurement, PTP time basis, and
`map -> odom` behavior on the vehicle are
`PENDING — requires Applanix and PandarXT hardware`.
