# Map-based LiDAR localization

## Start localization-only mode

Supply the map prefix or its `.mm` filename:

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch a1_mola_localization localization.launch.py \
  map:=${HOME}/.ros/a1_localization/maps/track/map
```

The launch refuses to start without the `.mm` file. It passes the metric map
and, when present, the matching `.simplemap` to MOLA. It passes these upstream
launch arguments:

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
`mola_msgs/srv/RelocalizeNearPose`.

After submitting the pose, activate MOLA-LO:

```zsh
ros2 service call /mola_runtime_param_set \
  mola_msgs/srv/MolaRuntimeParamSet \
  "{parameters: \"mola::LidarOdometry:lidar_odom:\\n  active: true\\n\"}"
```

Future GNSS/INS software may publish the same initial-pose request or use the
relocalization service. This package does not subscribe to GNSS or IMU and
does not prevent that later initializer from being added.

## Output interface

No A1 republisher is added because MOLA already publishes standard messages:

| Output | Type | Contract |
| --- | --- | --- |
| `/lidar_odometry/pose` | `nav_msgs/msg/Odometry` | `header.frame_id=map`, `child_frame_id=base_link`, pose and 6x6 pose covariance |
| `/lidar_odometry/pose_quality` | `std_msgs/msg/Float32` | last ICP quality in the MOLA localization update |
| `/lidar_odometry/localmap_points` | `sensor_msgs/msg/PointCloud2` | loaded `localmap` metric-map layer |
| `/tf` | `tf2_msgs/msg/TFMessage` | direct `map -> base_link` localization transform |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | input freshness, ICP quality, timing, map, and aggregate status |

The upstream Odometry publisher fills pose and pose covariance. It does not
populate measured twist/twist covariance in this path, so downstream fusion
must not interpret zero-initialized twist as a measured zero velocity.

## Restart test

Stop localization, run the same launch command with the same map, submit the
initial pose again, activate, and verify a continuous pose/quality stream.
Map load, lock, motion continuity, and restart are currently
`PENDING — requires PandarXT hardware`.
