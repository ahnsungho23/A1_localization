# Map-based localization

## Start localization

On the vehicle, start LiDAR, Applanix GNSS/INS, vehicle TF, MOLA, the merger
and RViz together:

```bash
source /opt/ros/humble/setup.bash
WS=$HOME/git/A1_localization
source $WS/install/setup.bash
MAP_DIR=$WS/maps/track_20261104      # an existing map folder (ls $WS/maps/)
ros2 launch a1_bringup localization.launch.py map:=$MAP_DIR/map
```

Without `map:=` the map is `<workspace>/maps/track/map`. Use the full-track
`map.mm` rebuilt from the simplemap ([mapping](mola_mapping.md#build-the-full-map-required)),
not the local map saved by `/map_save`. The launch refuses to start without the
`.mm` file. It passes the metric map and, when present, the
matching `.simplemap` to MOLA, with these upstream launch arguments:

```text
start_mapping_enabled = false
start_active = false
```

Thus incoming scans cannot modify the prebuilt map, and matching does not
begin before an initial pose is supplied.

## Set the initial pose and activate

In RViz, select **2D Pose Estimate** and publish a rough pose on
`/initialpose`. This is MOLA's official
`geometry_msgs/msg/PoseWithCovarianceStamped` relocalization input.
Alternatively, a caller can use `/relocalize_near_pose` with
`mola_msgs/srv/RelocalizeNearPose`. Keep the heading error small: in tests a
yaw error above about 5° kept a single ICP attempt from converging.

After submitting the pose, activate MOLA-LO:

```zsh
ros2 service call /mola_runtime_param_set \
  mola_msgs/srv/MolaRuntimeParamSet \
  "{parameters: \"mola::LidarOdometry:lidar_odom:\\n  active: true\\n\"}"
```

The GNSS/INS pose is not used for initialization; the initial pose is always
given by hand.

## Output interface

MOLA outputs:

| Output | Type | Contract |
| --- | --- | --- |
| `/lidar_odometry/pose` | `nav_msgs/msg/Odometry` | `header.frame_id=map`, `child_frame_id=base_link`, pose and 6x6 pose covariance, ~10 Hz |
| `/lidar_odometry/pose_quality` | `std_msgs/msg/Float32` | ICP quality of the pose just published |
| `/lidar_odometry/localmap_points` | `sensor_msgs/msg/PointCloud2` | loaded `localmap` metric-map layer |
| `/tf` | `tf2_msgs/msg/TFMessage` | `map -> odom` (REP-105) |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | input freshness, ICP quality, timing, map, and aggregate status |

MOLA fills pose and pose covariance only; the twist of `/lidar_odometry/pose`
is always zero and must not be read as a measured zero velocity.

Controller outputs (`localization_odom_merger`, ~100 Hz):

| Output | Type | Contract |
| --- | --- | --- |
| `/localization/odom` | `nav_msgs/msg/Odometry` | `map -> base_link` pose, INS velocity in `base_link` axes, heading (deg) in `twist.angular.x` |
| `/localization/accel` | `geometry_msgs/msg/AccelWithCovarianceStamped` | INS acceleration in `base_link` axes, 0.4 s mean |

Nothing is published on `/localization/*` before the first LO pose. Field
definitions are in [controller_interface.md](../controller_interface.md), and
how the merger combines GNSS and LO is in
[gnss_integration.md](../setup/gnss_integration.md#controller-output-localizationodom-localizationaccel).

## Restart

Stop localization, run the same launch command with the same map, submit the
initial pose again, activate, and check that `/lidar_odometry/pose`,
`/lidar_odometry/pose_quality` and `/localization/odom` stream continuously.
