# MOLA-LO installation

## Version policy

This workspace uses the official
[`MOLAorg/mola_lidar_odometry`](https://github.com/MOLAorg/mola_lidar_odometry)
repository as a Git submodule. It is pinned to tag `3.0.0`, commit
`4a8c7b70b59550a4ad66cfcc9be593bffe56c2aa`. ROS 2 Humble binary
dependencies are installed from the ROS package repository; upstream MOLA-LO
itself is built from the pinned source in this workspace.

Do not replace the submodule with an unmanaged nested clone or edit upstream
MOLA files for A1-specific tuning. A1 settings live in
`a1_localization`.

### `mola_metric_maps` 3.1.1 overlay

The apt `ros-humble-mola-metric-maps` 3.0.0 throws
`RKNN search requires nanoflann>=1.5.1` on Ubuntu 22.04 (system nanoflann
1.4.2), and MOLA-LO then discards every scan. Version 3.1.1 has a fallback for
old nanoflann, so the workspace builds that one package from source:

- `src/external/vendor/mola`: the `MOLAorg/mola` repository as a submodule,
  pinned to tag `3.1.1`. `src/external/vendor/COLCON_IGNORE` keeps colcon from
  building the other MOLA packages in it (the apt ones stay in use).
- `src/external/mola_metric_maps`: a symlink to
  `vendor/mola/mola_metric_maps`, the only package colcon picks up from it.

This replaces the separate `~/mola_humble_fix_ws` overlay.

## Clone and dependencies

```zsh
git clone --recurse-submodules \
  git@github.com:ahnsungho23/A1_localization.git a1_localization_ws
cd a1_localization_ws

source /opt/ros/humble/setup.zsh
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

If the repository was cloned without submodules:

```zsh
git submodule update --init --recursive
```

The dependency split:

| Source | Packages |
| --- | --- |
| Built from `src/external` (overrides apt) | `mola_lidar_odometry` 3.0.0, `mp2p_icp` 2.10.3, `mola_metric_maps` 3.1.1 |
| ROS binaries resolved by rosdep | MOLA core, ROS 2 bridge, launcher, visualization, state-estimation support, georeferencing, MRPT |
| Project packages | `src/a1/*` (`a1_localization` holds the MOLA launch and settings) |

A workspace package overrides the apt package of the same name.

- `mp2p_icp`: apt has 2.12.0, but the workspace builds the 2.10.3 submodule.
  The apt copy cannot be removed, because `mola-metric-maps`,
  `mola-georeferencing` and `mola-relocalization` depend on it.
- `mola_lidar_odometry`: the apt 3.0.0 copy is unused and nothing depends on
  it; it can be removed with `sudo apt remove ros-humble-mola-lidar-odometry`.

## Build

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

Confirm the selected versions:

```zsh
git submodule status --recursive
ros2 pkg prefix mola_lidar_odometry   # <workspace>/install/...
ros2 pkg prefix mola_metric_maps      # <workspace>/install/...
ros2 pkg prefix a1_localization
```

No ROS 2 Jazzy installation path is used by this workspace.
