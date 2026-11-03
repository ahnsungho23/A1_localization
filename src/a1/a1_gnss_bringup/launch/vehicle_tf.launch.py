from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from a1_extrinsics import load_extrinsics, static_tf_node  # noqa: E402

PACKAGE_NAME = "a1_gnss_bringup"


def launch_setup(context, *args, **kwargs):
    extrinsics_path = LaunchConfiguration("extrinsics").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context).strip()
    extrinsics = load_extrinsics(extrinsics_path)

    return [
        LogInfo(msg=f"[{PACKAGE_NAME}] vehicle extrinsics: {extrinsics_path}"),
        static_tf_node(base_frame, extrinsics["base_link_to_ins"]),
        static_tf_node(base_frame, extrinsics["base_link_to_lidar"]),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))

    return LaunchDescription([
        DeclareLaunchArgument(
            "extrinsics",
            default_value=str(package_share / "config" / "vehicle_extrinsics.yaml"),
            description="Measured base_link -> ins / hesai_lidar mounting poses",
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        OpaqueFunction(function=launch_setup),
    ])
