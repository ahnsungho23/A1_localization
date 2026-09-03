# Verification record

## Hardware boundary

The PandarXT is not physically connected for this integration milestone.
Hardware-dependent checks are never inferred from source code, old output, or
plausible sensor values. Their exact status is
`PENDING — requires PandarXT hardware`.

## Static procedure

```zsh
source /opt/ros/humble/setup.zsh
rosdep check --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.zsh

ros2 launch a1_mola_localization mapping.launch.py --show-args
ros2 launch a1_mola_localization localization.launch.py --show-args
python3 -m py_compile \
  src/a1_mola_localization/launch/*.launch.py \
  scripts/check_pandarxt_pointcloud.py
git diff --check
```

Launch/config inspection must confirm all of the following:

- official `lidar3d-gicp-optimize-twist.yaml` is selected
- `/lidar_points`, `hesai_lidar`, and `base_link` defaults match the existing
  driver contract
- `MOLA_OPTIMIZE_TWIST=true`
- missing per-point timestamps are not silently accepted
- IMU, GNSS, and external odometry inputs are disabled
- direct `map -> base_link` output is selected
- localization sets mapping and initial activity to false
- MOLA's optional `base_footprint -> base_link` publisher is disabled

## Results

| Check | Status | Evidence boundary |
| --- | --- | --- |
| Baseline pre-integration build | PASS | 2 packages built on ROS 2 Humble |
| Official MOLA source pin | PASS | tag `3.0.0`, commit `4a8c7b70b59550a4ad66cfcc9be593bffe56c2aa` |
| Dependency resolution | PASS | Humble rosdep keys resolved; official apt packages were unpacked into a temporary rootless underlay for validation |
| Persistent host dependency install | NOT INSTALLED — sudo required | Run the documented `rosdep install` command on the deployment host |
| Integrated default build | PASS | `colcon build --symlink-install`; 4 packages finished |
| Python/XML/YAML syntax | PASS | Integration launch files, diagnostic, package manifest, overlay and upstream pipeline parsed |
| PointCloud2 timestamp decoder | PASS | Synthetic little/big-endian and padded-row layouts decoded; no sensor values were assumed |
| MOLA module/class loading | PASS | BridgeROS2, LidarOdometry, KeyframePointCloudMap and MolaViz registered |
| Launch/config inspection | PASS | Both wrappers expand; mapping/localization actions construct; missing localization map is rejected |
| Existing Hesai source/config preservation | PASS | No tracked Hesai driver or `a1_lidar_bringup` source/config changes |
| Humble-only path inspection | PASS | No ROS 2 Jazzy installation path is referenced |
| `git diff --check` | PASS | No whitespace errors after final edits |
| Hesai standalone bringup regression | PENDING — requires PandarXT hardware | Requires UDP sensor input |
| `/lidar_points` live type/rate/count/frame | PENDING — requires PandarXT hardware | Requires a live cloud |
| Timestamp values/unit/span | PENDING — requires PandarXT hardware | Run `check_pandarxt_pointcloud.py` |
| TF lookup with installed sensor pose | PENDING — requires PandarXT hardware | Requires deployed robot TF |
| MOLA odometry trajectory change | PENDING — requires PandarXT hardware | Requires sensor motion |
| Mapping surface quality | PENDING — requires PandarXT hardware | Requires a driven mapping run |
| Map save/load | PENDING — requires PandarXT hardware | Requires a valid mapped environment |
| Localization-only lock/continuity | PENDING — requires PandarXT hardware | Requires a saved map and live scan |
| Initial pose workflow | PENDING — requires PandarXT hardware | Requires live relocalization |
| RViz live displays | PENDING — requires PandarXT hardware | Requires live ROS topics and TF |
| Restart on the same map | PENDING — requires PandarXT hardware | Requires a successful localization run |

## Hardware qualification sequence

1. Run the original `a1_lidar_bringup` alone and execute
   `./scripts/verify_lidar.sh <interface>`.
2. Run `./scripts/check_pandarxt_pointcloud.py` and retain its complete output.
3. Confirm `base_link -> hesai_lidar` with `tf2_echo`; do not add a second TF
   publisher.
4. Run mapping, inspect raw/deskewed/map surfaces, save, and verify both files.
5. Run localization-only, submit `/initialpose`, activate, and inspect pose,
   covariance, quality, TF, and diagnostics while moving.
6. Stop and restart against the same unmodified map.

Only replace a PENDING row with PASS or FAIL after that exact observation has
been collected.
