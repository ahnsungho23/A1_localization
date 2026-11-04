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

```bash
source /opt/ros/humble/setup.bash
WS=$HOME/git/A1_localization
source $WS/install/setup.bash
MAP_DIR=$WS/maps/track_20261104      # new folder per mapping run
ros2 launch a1_bringup mapping.launch.py map_output_path:=$MAP_DIR/map
```

`WS` and `MAP_DIR` must be set to the same values in every terminal used below.

Mapping starts immediately (`start_active=true`). MOLA publishes
`map -> odom` (REP-105), and every keyframe of the `.simplemap` stores the
nearest `/a1_gnss/navsat` fix for georeferencing.

The project defaults are `mola_gui:=false` and `rviz:=true`. Mapping,
map save/load services, and localization are provided by the non-GUI MOLA
modules and do not require the MOLA GUI.

Without `map_output_path` the prefix is `<workspace>/maps/track/map`, and an
existing map there is overwritten.

If the PandarXT or Applanix driver is already running, prevent a duplicate
with `start_lidar_driver:=false` or `start_gnss_driver:=false`.

Other useful launch arguments include `lidar_topic`, `lidar_frame`,
`base_frame`, `pipeline`, `rviz`, `mola_gui`, `timestamp_type`, and
`publish_deskewed_scans`.

## Save the map

Stop the vehicle, then save through MOLA's service before stopping the launch
with Ctrl-C. The service takes a prefix, not a `.pcd` filename, and should
match `map_output_path`:

```bash
ros2 service call /map_save mola_msgs/srv/MapSave "{map_path: '$MAP_DIR/map'}"
```

Verify that the response has `success: true`. The service writes:

```text
map.mm          # MOLA's local map only (see below)
map.simplemap   # all keyframes, observations and GNSS fixes
```

The launch also supplies those output filenames to MOLA so a clean shutdown
can save the final map, but a large map may not finish saving before the
process is stopped. Treat the service response and existence of both files as
the authoritative save check.

## Build the full map (required)

The saved `map.mm` is MOLA-LO's local map (`save_final_local_map`), a
`mola::KeyframePointCloudMap` that deletes keyframes farther than
`max(100 m, 1.5 × observation radius)` from the vehicle
(`remove_frames_farther_than`). It therefore only covers ~100–150 m around the
point where mapping stopped. The `.simplemap` keeps every keyframe, so rebuild
the full map from it:

```bash
cd $MAP_DIR
mv map.mm map_local.mm
sm2mm -i map.simplemap -o map.mm \
  -p $WS/src/external/mp2p_icp/mp2p_icp_core/demos/sm2mm_pointcloud_voxelize_keyframe_map.yaml \
  -l libmola_metric_maps.so
mm-info map.mm          # extent should cover the whole track; also: mm-viewer map.mm
```

The pipeline creates the `localmap` layer as a `KeyframePointCloudMap` (the
layer and class localization expects) with keyframe deletion disabled
(`remove_frames_farther_than: 0`). It is an upstream demo configuration;
check the first result visually and tune it if needed.

## Georeferencing (after the full map)

MOLA attaches to each keyframe the `/a1_gnss/navsat` fix closest in time, if it
is within 1 s of the scan (`save_gnss_max_age`). Georeferencing therefore needs
GNSS to be running during mapping, stamps on the same clock
(`time_source: now`), and GSOF #50 for the fix covariance.

```bash
sm-cli info map.simplemap        # check that gps observations are present
mola-sm-georeferencing -i map.simplemap --write-into map.mm -o map.georef
```

Run it after the full-map rebuild; otherwise the georeference is written into
the local map that gets replaced. The controller outputs are in the `map`
frame and do not change with georeferencing; it is only needed when GNSS
coordinates are used elsewhere.

## Map files

| File | Created | Content |
| --- | --- | --- |
| `map.simplemap` | save | keyframe poses, LiDAR scans, GNSS fixes |
| `map_local.mm` | save, renamed | local map around the stop point (kept for reference) |
| `map.mm` | full-map rebuild | full-track map used for localization; includes the georeference after that step |
| `map.georef` | georeferencing | map ↔ latitude/longitude (ENU/UTM) transform |

## RViz inspection

The reused official MOLA RViz configuration is remapped to `/lidar_points` and
shows raw points, `/lidar_odometry/localmap_points`, odometry history, TF, and
the `/initialpose` tool. Enable `publish_deskewed_scans:=true` to publish
`/lidar_odometry/deskewed_scan_points`; this costs additional processing.

Accept a map only when surfaces are thin and stable rather than doubled or
ghosted.
