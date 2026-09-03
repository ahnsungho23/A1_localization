from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
import yaml


PACKAGE_NAME = "a1_mola_localization"
SUPPORTED_MODES = {"mapping", "localization"}


def _load_defaults(package_share):
    config_path = package_share / "config" / "pandarxt_lidar_only.yaml"
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict) or "a1_mola" not in config:
        raise RuntimeError(f"Invalid A1 MOLA configuration: {config_path}")
    return config["a1_mola"]


def _parse_bool(name, value):
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise RuntimeError(f"{name} must be true or false, got {value!r}")


def _parse_pose(value):
    components = [item.strip() for item in value.strip().strip("[]").split(",")]
    if len(components) != 6 or any(not item for item in components):
        raise RuntimeError(
            "lidar_pose must contain [x,y,z,yaw_deg,pitch_deg,roll_deg]"
        )
    try:
        [float(item) for item in components]
    except ValueError as exc:
        raise RuntimeError("lidar_pose components must be numeric") from exc
    return components


def _map_prefix(value):
    path = Path(value).expanduser()
    if path.suffix in {".mm", ".simplemap"}:
        path = path.with_suffix("")
    return path.resolve()


def _launch_setup(context, package_share, defaults):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in SUPPORTED_MODES:
        raise RuntimeError(
            f"mode must be one of {sorted(SUPPORTED_MODES)}, got {mode!r}"
        )

    lidar_topic = LaunchConfiguration("lidar_topic").perform(context).strip()
    lidar_frame = LaunchConfiguration("lidar_frame").perform(context).strip()
    base_frame = LaunchConfiguration("base_frame").perform(context).strip()
    pipeline = Path(
        LaunchConfiguration("pipeline").perform(context)
    ).expanduser().resolve()

    if not lidar_topic or not lidar_frame or not base_frame:
        raise RuntimeError("lidar_topic, lidar_frame, and base_frame must be non-empty")
    if not pipeline.is_file():
        raise FileNotFoundError(f"MOLA pipeline does not exist: {pipeline}")

    start_driver = _parse_bool(
        "start_lidar_driver",
        LaunchConfiguration("start_lidar_driver").perform(context),
    )
    use_fixed_pose = _parse_bool(
        "use_fixed_lidar_pose",
        LaunchConfiguration("use_fixed_lidar_pose").perform(context),
    )
    publish_deskewed = _parse_bool(
        "publish_deskewed_scans",
        LaunchConfiguration("publish_deskewed_scans").perform(context),
    )

    actions = [
        LogInfo(
            msg=(
                f"[a1_mola_localization] mode={mode}, input={lidar_topic} "
                f"(expected frame={lidar_frame}), base={base_frame}, "
                f"pipeline={pipeline}"
            )
        ),
        SetEnvironmentVariable(
            "MOLA_OPTIMIZE_TWIST", str(defaults["optimize_twist"]).lower()
        ),
        SetEnvironmentVariable(
            "MOLA_IGNORE_NO_POINT_STAMPS",
            str(not defaults["require_point_timestamps"]).lower(),
        ),
        SetEnvironmentVariable(
            "MOLA_SCAN_POINT_STAMPS_ADJUST_METHOD",
            str(defaults["point_stamp_adjust_method"]),
        ),
        SetEnvironmentVariable(
            "MOLA_LO_PUBLISH_DESKEWED_SCANS", str(publish_deskewed).lower()
        ),
        # Empty disables the upstream optional base_footprint->base_link
        # static publisher. The robot's existing TF owner remains authoritative.
        SetEnvironmentVariable("MOLA_TF_FOOTPRINT_LINK", ""),
    ]

    if use_fixed_pose:
        pose = _parse_pose(LaunchConfiguration("lidar_pose").perform(context))
        pose_names = (
            "LIDAR_POSE_X",
            "LIDAR_POSE_Y",
            "LIDAR_POSE_Z",
            "LIDAR_POSE_YAW",
            "LIDAR_POSE_PITCH",
            "LIDAR_POSE_ROLL",
        )
        actions.extend(
            SetEnvironmentVariable(name, value)
            for name, value in zip(pose_names, pose)
        )

    if start_driver:
        lidar_launch = (
            Path(get_package_share_directory("a1_lidar_bringup"))
            / "launch"
            / "pandarxt.launch.py"
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lidar_launch)),
                launch_arguments={
                    "timestamp_type": LaunchConfiguration("timestamp_type")
                }.items(),
            )
        )

    is_mapping = mode == "mapping"
    mola_arguments = {
        "lidar_topic_name": lidar_topic,
        "lidar_topic_type": "PointCloud2",
        "lidar_qos_reliability": str(defaults["lidar_qos_reliability"]),
        "lidar_qos_depth": str(defaults["lidar_qos_depth"]),
        "mola_tf_base_link": base_frame,
        "mola_lo_reference_frame": "map",
        "mola_state_estimator_reference_frame": "map",
        "mola_bridge_odometry_frame": "odom",
        "mola_lo_pipeline": str(pipeline),
        "mola_deskew_method": str(defaults["deskew_method"]),
        "ignore_lidar_pose_from_tf": str(use_fixed_pose),
        "use_imu_for_lio": "False",
        "imu_gravity_correction": "False",
        "imu_topic_name": "",
        "gnss_topic_name": "",
        "gpsfix_topic_name": "",
        "gnss_mode": "none",
        "forward_ros_tf_odom_to_mola": "False",
        "odom_topic_name": "",
        "use_state_estimator": "False",
        "publish_localization_following_rep105": "False",
        "start_mapping_enabled": str(is_mapping),
        "start_active": str(is_mapping),
        "generate_simplemap": str(is_mapping),
        "use_mola_gui": LaunchConfiguration("mola_gui"),
        "use_rviz": LaunchConfiguration("rviz"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    if is_mapping:
        output_prefix = _map_prefix(
            LaunchConfiguration("map_output_path").perform(context)
        )
        output_prefix.parent.mkdir(parents=True, exist_ok=True)
        actions.extend([
            LogInfo(
                msg=f"[a1_mola_localization] map prefix: {output_prefix}"
            ),
            SetEnvironmentVariable("MOLA_SAVE_MM", f"{output_prefix}.mm"),
            SetEnvironmentVariable(
                "MOLA_SIMPLEMAP_OUTPUT", f"{output_prefix}.simplemap"
            ),
        ])
    else:
        map_value = LaunchConfiguration("map").perform(context).strip()
        if not map_value:
            raise RuntimeError(
                "localization mode requires map:=<map prefix or .mm path>"
            )
        input_prefix = _map_prefix(map_value)
        mm_file = Path(f"{input_prefix}.mm")
        simplemap_file = Path(f"{input_prefix}.simplemap")
        if not mm_file.is_file():
            raise FileNotFoundError(f"Required MOLA metric map not found: {mm_file}")

        actions.append(
            LogInfo(
                msg=(
                    f"[a1_mola_localization] localization map: {mm_file}; "
                    "mapping is disabled and MOLA starts inactive"
                )
            )
        )
        mola_arguments["mola_initial_map_mm_file"] = str(mm_file)
        if simplemap_file.is_file():
            mola_arguments["mola_initial_map_sm_file"] = str(simplemap_file)

    upstream_launch = (
        Path(get_package_share_directory("mola_lidar_odometry"))
        / "share"
        / "mola_lidar_odometry"
        / "ros2-launchs"
        / "ros2-lidar-odometry.launch.py"
    )
    if not upstream_launch.is_file():
        # Source and binary installs both normally resolve to <prefix>/share.
        upstream_launch = (
            Path(get_package_share_directory("mola_lidar_odometry"))
            / "ros2-launchs"
            / "ros2-lidar-odometry.launch.py"
        )
    if not upstream_launch.is_file():
        raise FileNotFoundError(f"Upstream MOLA launch file not found: {upstream_launch}")

    actions.append(
        GroupAction([
            # The upstream RViz file uses its demo topic name. Keep the file
            # unchanged and remap that display to the configured PandarXT topic.
            SetRemap(src="/lidar1_points", dst=lidar_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(upstream_launch)),
                launch_arguments=mola_arguments.items(),
            ),
        ])
    )
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    defaults = _load_defaults(package_share)
    mola_share = Path(get_package_share_directory("mola_lidar_odometry"))
    default_pipeline = mola_share / "pipelines" / str(defaults["pipeline"])
    if not default_pipeline.is_file():
        default_pipeline = (
            mola_share
            / "share"
            / "mola_lidar_odometry"
            / "pipelines"
            / str(defaults["pipeline"])
        )

    default_map_prefix = (
        Path.home() / ".ros" / "a1_localization" / "maps" / "track" / "map"
    )

    arguments = [
        DeclareLaunchArgument(
            "mode", default_value="mapping", description="mapping or localization"
        ),
        DeclareLaunchArgument(
            "lidar_topic", default_value=str(defaults["lidar_topic"])
        ),
        DeclareLaunchArgument(
            "lidar_frame", default_value=str(defaults["lidar_frame"])
        ),
        DeclareLaunchArgument(
            "base_frame", default_value=str(defaults["base_frame"])
        ),
        DeclareLaunchArgument("pipeline", default_value=str(default_pipeline)),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("mola_gui", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_lidar_driver", default_value="true"),
        DeclareLaunchArgument(
            "timestamp_type",
            default_value="1",
            description="Hesai: 0=LiDAR/PTP time, 1=host receive time",
        ),
        DeclareLaunchArgument(
            "use_fixed_lidar_pose",
            default_value="false",
            description=(
                "Use lidar_pose instead of looking up base_frame->lidar_frame in TF"
            ),
        ),
        DeclareLaunchArgument(
            "lidar_pose",
            default_value="[0,0,0,0,0,0]",
            description="[x,y,z,yaw_deg,pitch_deg,roll_deg]; opt-in only",
        ),
        DeclareLaunchArgument(
            "publish_deskewed_scans",
            default_value=str(defaults["publish_deskewed_scans"]).lower(),
        ),
        DeclareLaunchArgument(
            "map_output_path",
            default_value=str(default_map_prefix),
            description="Mapping output prefix without .mm/.simplemap",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Localization map prefix or .mm path",
        ),
    ]

    return LaunchDescription(
        arguments
        + [
            OpaqueFunction(
                function=_launch_setup,
                args=[package_share, defaults],
            )
        ]
    )
