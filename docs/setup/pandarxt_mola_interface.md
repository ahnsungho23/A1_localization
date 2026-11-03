# PandarXT to MOLA-LO interface

## Data contract

The existing Hesai driver remains the only owner of PandarXT acquisition.
MOLA subscribes directly; there is no republisher or field-conversion node.

| Property | Contract |
| --- | --- |
| Topic | `/lidar_points` |
| Type | `sensor_msgs/msg/PointCloud2` |
| Header frame | `hesai_lidar` |
| Required fields | `x`, `y`, `z`, `intensity`, `ring`, `timestamp` |
| `timestamp` type/unit | `FLOAT64`, seconds |
| Nominal scan rate | 10 Hz from the current PandarXT configuration |

The current driver source exports `x/y/z/intensity/ring/timestamp` with a
26-byte point step. Its PandarXT parser calculates firetime-adjusted
nanosecond values internally, but `LidarPointXYZIRT.timestamp` is populated
with the packet timestamp. Consequently, points in the same packet can have
identical exported timestamps. Do not describe this stream as having a unique
timestamp for every physical firing until a hardware capture proves it.

The Humble `mrpt_ros2bridge` used by MOLA recognizes `timestamp`, `time`, or
`t`; it accepts FLOAT32/FLOAT64 seconds and converts absolute timestamps to
scan-relative values before storing them in the MOLA point map. The existing
Hesai schema therefore needs no adapter. Whether packet-level time resolution
is sufficient for high-speed driving remains a hardware validation item.

## Timestamp modes

`a1_lidar_bringup` keeps its existing selector:

- `timestamp_type:=1`: host receive time; current default while PTP is not
  locked
- `timestamp_type:=0`: LiDAR time; use only after a PTP grandmaster and actual
  PandarXT lock are verified

The cloud header uses the frame-start timestamp. MOLA is configured with
`TimestampAdjustMethod::MiddleIsZero` before applying constant-velocity
deskew.

## Deskew

The selected upstream pipeline is
`lidar3d-gicp-optimize-twist.yaml`, with these A1 overrides:

- `MotionCompensationMethod::Linear`
- `MOLA_OPTIMIZE_TWIST=true`
- `MOLA_IGNORE_NO_POINT_STAMPS=false`
- IMU gravity correction disabled
- IMU and external odometry topic inputs disabled; GNSS (`/a1_gnss/navsat`) is
  only recorded into the simplemap, not used for pose estimation

Missing point timestamps are therefore a hard failure instead of silently
disabling deskew. MOLA estimates twist from LiDAR registration; PandarXT IMU
is not used.

## TF ownership

The Hesai bringup publishes no TF. `a1_gnss_bringup` publishes the measured
static transforms from `vehicle_extrinsics.yaml`, and MOLA looks up
`base_link -> hesai_lidar` in TF:

```text
map ──[MOLA]──▶ odom ──[gnss_odom_relay]──▶ base_link ──[static]──▶ hesai_lidar
                                                     └──[static]──▶ ins
```

MOLA runs in REP-105 mode and publishes `map -> odom`; the Applanix relay owns
`odom -> base_link`. The integration launch also disables MOLA's optional
`base_footprint -> base_link` static transform to avoid duplicate ownership.

## Live diagnostic

With the driver running, inspect one full cloud:

```zsh
./tools/analysis/check_pandarxt_pointcloud.py \
  --topic /lidar_points \
  --expected-frame hesai_lidar
```

The script checks field types, finite/non-constant timestamps, scan span,
header alignment, and reports how many timestamps are unique.
