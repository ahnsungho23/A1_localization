"""Vehicle localization: PandarXT + Applanix + MOLA localization + /localization/*.

map defaults to the mapping output (<workspace>/maps/track/map); override with
map:=<prefix or .mm path>. Any other a1_localization mola.launch.py argument
can be appended as well.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    common_launch = (
        Path(get_package_share_directory("a1_localization"))
        / "launch"
        / "mola.launch.py"
    )

    # Same default as the mapping output of mola.launch.py: with
    # --symlink-install this file resolves into <workspace>/src.
    workspace = Path(__file__).resolve().parents[4]
    if (workspace / "src" / "a1" / "a1_bringup").is_dir():
        default_map_dir = workspace / "maps"
    else:
        default_map_dir = Path.home() / ".ros" / "a1_localization" / "maps"

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=str(default_map_dir / "track" / "map"),
            description="Localization map prefix or .mm path",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(common_launch)),
            launch_arguments={
                "mode": "localization",
                "map": LaunchConfiguration("map"),
            }.items(),
        ),
    ])
