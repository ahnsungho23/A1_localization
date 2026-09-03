# LiDAR-only mapping

## Prerequisites

Build and source the workspace, configure the PandarXT host network, and make
sure the calibrated `base_link -> hesai_lidar` TF is owned by the robot model.
See [PandarXT to MOLA-LO interface](pandarxt_mola_interface.md) before using a
fixed pose.

## Start mapping

The default command includes the existing PandarXT launch, MOLA-LO, and RViz:

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch a1_mola_localization mapping.launch.py
```

The project defaults are `mola_gui:=false` and `rviz:=true`. Mapping,
map save/load services, and localization are provided by the non-GUI MOLA
modules and do not require the MOLA GUI.

The default map prefix is
`~/.ros/a1_localization/maps/track/map`. To choose another prefix:

```zsh
ros2 launch a1_mola_localization mapping.launch.py \
  map_output_path:=${HOME}/a1_maps/track/map
```

If the PandarXT driver is already running, prevent a duplicate driver:

```zsh
ros2 launch a1_mola_localization mapping.launch.py \
  start_lidar_driver:=false
```

Useful launch arguments include `lidar_topic`, `lidar_frame`, `base_frame`,
`pipeline`, `rviz`, `mola_gui`, `map_output_path`, `timestamp_type`, and
`publish_deskewed_scans`.

## Save the map

MOLA's official save interface takes a prefix, not a `.pcd` filename:

```zsh
ros2 service call /map_save mola_msgs/srv/MapSave \
  "{map_path: '${HOME}/.ros/a1_localization/maps/track/map'}"
```

Verify that the response has `success: true`. The expected files are:

```text
map.mm          # metric map used for localization
map.simplemap   # keyframes and observations for later map processing
```

The launch also supplies those output filenames to MOLA so a clean shutdown
can save the final map. Treat the service response and existence of both files
as the authoritative save check.

## RViz inspection

The reused official MOLA RViz configuration is remapped to `/lidar_points` and
shows raw points, `/lidar_odometry/localmap_points`, odometry history, TF, and
the `/initialpose` tool. Enable `publish_deskewed_scans:=true` to publish
`/lidar_odometry/deskewed_scan_points`; this costs additional processing.

Thin, stable surfaces rather than doubled/ghosted surfaces are required before
accepting a map. This visual and motion-dependent result is currently
`PENDING — requires PandarXT hardware`.
