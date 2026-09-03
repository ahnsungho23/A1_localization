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
`a1_mola_localization`.

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

The dependency split is intentional:

- source-pinned: `mola_lidar_odometry`
- ROS binaries resolved by rosdep: MOLA core, ROS 2 bridge, launcher,
  visualization, state-estimation support, MRPT, and mp2p_icp
- project-owned overlay: `a1_mola_localization`

## Build

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

Confirm the selected versions:

```zsh
git submodule status --recursive
ros2 pkg prefix mola_lidar_odometry
ros2 pkg prefix a1_mola_localization
```

No ROS 2 Jazzy installation path is used by this workspace.
