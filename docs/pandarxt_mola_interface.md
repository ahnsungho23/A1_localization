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

## LiDAR-only deskew

The selected upstream pipeline is
`lidar3d-gicp-optimize-twist.yaml`, with these A1 overrides:

- `MotionCompensationMethod::Linear`
- `MOLA_OPTIMIZE_TWIST=true`
- `MOLA_IGNORE_NO_POINT_STAMPS=false`
- IMU gravity correction disabled
- IMU, GNSS, and external odometry topic inputs disabled

Missing point timestamps are therefore a hard failure instead of silently
disabling deskew. MOLA estimates twist from LiDAR registration; PandarXT IMU
is not used.

## TF ownership

The existing Hesai bringup publishes no TF. The normal integration path
expects another robot-description owner to publish the calibrated transform:

```text
map -> base_link -> hesai_lidar
```

MOLA publishes `map -> base_link` directly. REP-105 mode is intentionally off
because this workspace has no verified external `odom -> base_link` source.
The integration launch also disables MOLA's optional
`base_footprint -> base_link` static transform to avoid duplicate ownership.

If no robot TF publisher is available, an explicit calibrated sensor pose can
be supplied without publishing a new TF:

```zsh
use_fixed_lidar_pose:=true \
lidar_pose:="[x,y,z,yaw_deg,pitch_deg,roll_deg]"
```

The identity default is ignored unless fixed-pose mode is explicitly enabled.
Do not enable it with zero values unless that is the measured mounting pose.

## Live diagnostic

With the driver running, inspect one full cloud:

```zsh
./scripts/check_pandarxt_pointcloud.py \
  --topic /lidar_points \
  --expected-frame hesai_lidar
```

The script checks field types, finite/non-constant timestamps, scan span,
header alignment, and reports how many timestamps are unique. With the sensor
currently disconnected, its result is:
`PENDING — requires PandarXT hardware`.
