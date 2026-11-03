from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from a1_extrinsics import load_extrinsics, static_tf_node  # noqa: E402

PACKAGE_NAME = "a1_gnss_bringup"
DRIVER_NODE_NAME = "gsof_client"  # must match the key in the driver YAML


def _parse_bool(name, value):
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise RuntimeError(f"{name} must be true or false, got {value!r}")


def _load_driver_params(path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"GSOF driver config not found: {path}")
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    try:
        return path, config[DRIVER_NODE_NAME]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f"{path}: expected {DRIVER_NODE_NAME}.ros__parameters") from exc


def launch_setup(context, *args, **kwargs):
    base_frame = LaunchConfiguration("base_frame").perform(context).strip()
    odom_frame = LaunchConfiguration("odom_frame").perform(context).strip()
    use_sim_time = _parse_bool(
        "use_sim_time", LaunchConfiguration("use_sim_time").perform(context))
    start_driver = _parse_bool(
        "start_driver", LaunchConfiguration("start_driver").perform(context))
    start_vehicle_tf = _parse_bool(
        "start_vehicle_tf", LaunchConfiguration("start_vehicle_tf").perform(context))

    extrinsics_path = LaunchConfiguration("extrinsics").perform(context)
    extrinsics = load_extrinsics(extrinsics_path)
    ins = extrinsics["base_link_to_ins"]

    driver_config, driver_params = _load_driver_params(
        LaunchConfiguration("driver_config").perform(context))

    # The relay owns odom -> base_link; a driver TF would give "ins" a second parent.
    if driver_params.get("publish_tf", True):
        raise RuntimeError(f"{driver_config}: publish_tf must be false")
    if driver_params.get("child_frame") != ins["frame_id"]:
        raise RuntimeError(
            f"{driver_config}: child_frame {driver_params.get('child_frame')!r} "
            f"must match base_link_to_ins.frame_id {ins['frame_id']!r} in {extrinsics_path}")
    time_source = driver_params.get("time_source")
    time_conversion = {"gps": "gps_to_utc", "now": "none"}.get(time_source)
    if time_conversion is None:
        raise RuntimeError(
            f"{driver_config}: time_source must be 'gps' or 'now', got {time_source!r}")

    actions = [
        LogInfo(msg=(
            f"[{PACKAGE_NAME}] driver config: {driver_config}, extrinsics: "
            f"{extrinsics_path}, time_source={time_source} -> {time_conversion}")),
    ]

    if start_driver:
        ip = LaunchConfiguration("ip").perform(context).strip() or str(
            driver_params.get("ip", "")).strip()
        if not ip:
            raise RuntimeError(
                "Applanix IP is not set: pass ip:=<device ip> or set it in "
                f"{driver_config}")
        overrides = {"ip": ip}
        port = LaunchConfiguration("port").perform(context).strip()
        if port:
            overrides["port"] = int(port)

        actions.append(Node(
            package="trimble_driver_ros",
            executable="gsof_client_node",
            name=DRIVER_NODE_NAME,
            parameters=[str(driver_config), overrides],
            output="screen",
        ))

    if start_vehicle_tf:
        actions += [
            static_tf_node(base_frame, ins),
            static_tf_node(base_frame, extrinsics["base_link_to_lidar"]),
        ]

    actions.append(Node(
        package=PACKAGE_NAME,
        executable="gnss_odom_relay.py",
        name="gnss_odom_relay",
        parameters=[{
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "ins_frame": ins["frame_id"],
            "time_conversion": time_conversion,
            "base_to_ins_xyz": [ins["x"], ins["y"], ins["z"]],
            "base_to_ins_rpy_deg": [ins["roll"], ins["pitch"], ins["yaw"]],
            "input_odom_topic": f"/{DRIVER_NODE_NAME}/odom",
            "input_navsat_topic": f"/{DRIVER_NODE_NAME}/navsat",
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    ))
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))

    return LaunchDescription([
        DeclareLaunchArgument(
            "ip", default_value="",
            description="Applanix device IP (overrides the driver YAML)"),
        DeclareLaunchArgument(
            "port", default_value="",
            description="GSOF TCP port (overrides the driver YAML)"),
        DeclareLaunchArgument(
            "driver_config",
            default_value=str(package_share / "config" / "gsof_client.yaml")),
        DeclareLaunchArgument(
            "extrinsics",
            default_value=str(package_share / "config" / "vehicle_extrinsics.yaml")),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument(
            "start_driver", default_value="true",
            description="false: replay /gsof_client/* from a bag through the relay"),
        DeclareLaunchArgument(
            "start_vehicle_tf", default_value="true",
            description="Publish the measured base_link -> ins / hesai_lidar static TFs"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
