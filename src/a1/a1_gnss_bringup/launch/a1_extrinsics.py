"""Shared loader for config/vehicle_extrinsics.yaml (imported by the launch files)."""

import math
from pathlib import Path

from launch_ros.actions import Node
import yaml

POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
SECTIONS = ("base_link_to_ins", "base_link_to_lidar")


def load_extrinsics(path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Vehicle extrinsics file not found: {path}")

    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    extrinsics = {}
    for section in SECTIONS:
        entry = (config or {}).get(section)
        if not isinstance(entry, dict) or not entry.get("frame_id"):
            raise RuntimeError(f"{path}: '{section}' with a frame_id is required")

        missing = [key for key in POSE_KEYS if entry.get(key) is None]
        if missing:
            raise RuntimeError(
                f"{path}: {section} is not measured yet (null: {', '.join(missing)}). "
                "Fill in the measured mounting pose before launching."
            )
        try:
            pose = {key: float(entry[key]) for key in POSE_KEYS}
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"{path}: {section} values must be numeric") from exc

        extrinsics[section] = {"frame_id": str(entry["frame_id"]), **pose}
    return extrinsics


def static_tf_node(parent_frame, pose):
    """static_transform_publisher for parent_frame -> pose['frame_id'] (angles in deg)."""
    arguments = []
    for key in ("x", "y", "z"):
        arguments += [f"--{key}", str(pose[key])]
    for key in ("roll", "pitch", "yaw"):
        arguments += [f"--{key}", str(math.radians(pose[key]))]
    arguments += ["--frame-id", parent_frame, "--child-frame-id", pose["frame_id"]]

    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{parent_frame}_to_{pose['frame_id']}",
        arguments=arguments,
        output="screen",
    )
