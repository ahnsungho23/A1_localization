# Mapping

## Prerequisites

Build and source the workspace, configure the PandarXT host network, and check
that the Applanix GSOF #49 and #50 outputs are enabled
([gnss_integration.md](../setup/gnss_integration.md)). The vehicle static TFs
(`base_link -> hesai_lidar`, `base_link -> ins`) come from
`a1_gnss_bringup/config/vehicle_extrinsics.yaml`.

## Start mapping

The command starts the PandarXT driver, the Applanix driver and relay, the
vehicle TFs, MOLA-LO in mapping mode, the localization merger and RViz:

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch a1_bringup mapping.launch.py
```

Mapping starts immediately (`start_active=true`). MOLA publishes
`map -> odom` (REP-105), and every keyframe of the `.simplemap` stores the
nearest `/a1_gnss/navsat` fix for georeferencing.

The project defaults are `mola_gui:=false` and `rviz:=true`. Mapping,
map save/load services, and localization are provided by the non-GUI MOLA
modules and do not require the MOLA GUI.

The default map prefix is `<workspace>/maps/track/map`, and an existing map
there is overwritten. To choose another prefix:

```zsh
ros2 launch a1_bringup mapping.launch.py \
  map_output_path:=${HOME}/a1_localization_ws/maps/track_1105/map
```

If the PandarXT or Applanix driver is already running, prevent a duplicate
with `start_lidar_driver:=false` or `start_gnss_driver:=false`.

Other useful launch arguments include `lidar_topic`, `lidar_frame`,
`base_frame`, `pipeline`, `rviz`, `mola_gui`, `timestamp_type`, and
`publish_deskewed_scans`.

## Save the map

Stop the vehicle, then save through MOLA's service before stopping the launch
with Ctrl-C. The service takes a prefix, not a `.pcd` filename, and should
match `map_output_path`:

```zsh
ros2 service call /map_save mola_msgs/srv/MapSave \
  "{map_path: '$HOME/a1_localization_ws/maps/track/map'}"
```

Verify that the response has `success: true`. The expected files are:

```text
map.mm          # metric map used for localization
map.simplemap   # keyframes, observations and GNSS fixes
```

The launch also supplies those output filenames to MOLA so a clean shutdown
can save the final map, but a large map may not finish saving before the
process is stopped. Treat the service response and existence of both files as
the authoritative save check.

## Georeferencing

To relate the map to latitude/longitude, run after saving:

```zsh
cd ~/a1_localization_ws/maps/track
mola-sm-georeferencing -i map.simplemap --write-into map.mm -o map.georef
```

The controller outputs are in the `map` frame and do not change with
georeferencing; it is only needed when GNSS coordinates are used elsewhere.

## RViz inspection

The reused official MOLA RViz configuration is remapped to `/lidar_points` and
shows raw points, `/lidar_odometry/localmap_points`, odometry history, TF, and
the `/initialpose` tool. Enable `publish_deskewed_scans:=true` to publish
`/lidar_odometry/deskewed_scan_points`; this costs additional processing.

Accept a map only when surfaces are thin and stable rather than doubled or
ghosted.
