from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    common_launch = (
        Path(get_package_share_directory("a1_mola_localization"))
        / "launch"
        / "mola.launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(common_launch)),
            launch_arguments={"mode": "mapping"}.items(),
        )
    ])
