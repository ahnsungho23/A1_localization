from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "a1_lidar_bringup"
FIRETIME_FILENAME = "XT32_Firetime_Correction_File.csv"


def launch_setup(context, *args, **kwargs):
    package_share = Path(get_package_share_directory(PACKAGE_NAME))

    template_path = Path(
        LaunchConfiguration("config_template").perform(context)
    ).expanduser().resolve()
    timestamp_type = LaunchConfiguration("timestamp_type").perform(context)

    if timestamp_type not in {"0", "1"}:
        raise RuntimeError(
            "timestamp_type must be 0 (LiDAR/PTP time) "
            "or 1 (host receive time)"
        )

    firetime_path = (
        package_share / "calibration" / FIRETIME_FILENAME
    ).resolve()

    if not template_path.is_file():
        raise FileNotFoundError(
            f"LiDAR config template not found: {template_path}"
        )

    if not firetime_path.is_file():
        raise FileNotFoundError(
            f"Firetime correction file not found: {firetime_path}"
        )

    config_text = template_path.read_text(encoding="utf-8")

    replacements = {
        "__FIRETIMES_PATH__": str(firetime_path),
        "__TIMESTAMP_TYPE__": timestamp_type,
    }

    for token, value in replacements.items():
        if token not in config_text:
            raise RuntimeError(
                f"Required token {token} is missing from {template_path}"
            )
        config_text = config_text.replace(token, value)

    ros_home_value = os.environ.get("ROS_HOME")
    ros_home = Path(
        ros_home_value if ros_home_value else Path.home() / ".ros"
    ).expanduser().resolve()
    runtime_directory = ros_home / "a1_localization"
    runtime_directory.mkdir(parents=True, exist_ok=True)

    runtime_config = runtime_directory / "pandarxt_runtime.yaml"
    runtime_config.write_text(config_text, encoding="utf-8")

    print(f"[a1_lidar_bringup] config: {runtime_config}")
    print(f"[a1_lidar_bringup] firetime: {firetime_path}")
    print(f"[a1_lidar_bringup] timestamp_type: {timestamp_type}")

    return [
        Node(
            package="hesai_ros_driver",
            executable="hesai_ros_driver_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"config_path": str(runtime_config)}
            ],
        )
    ]


def generate_launch_description():
    package_share = Path(
        get_package_share_directory(PACKAGE_NAME)
    )

    default_template = (
        package_share / "config" / "pandarxt.yaml.in"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_template",
            default_value=str(default_template),
            description="PandarXT configuration template",
        ),
        DeclareLaunchArgument(
            "timestamp_type",
            default_value="1",
            description=(
                "0: LiDAR timestamp with locked PTP, "
                "1: host receive timestamp"
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
