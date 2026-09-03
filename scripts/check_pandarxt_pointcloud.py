#!/usr/bin/env python3
"""Validate one live PandarXT PointCloud2 message without changing ROS state."""

import argparse
import math
import struct
import sys
import time


PENDING = "PENDING — requires PandarXT hardware"
EXPECTED_FIELDS = {
    "x": 7,  # PointField.FLOAT32
    "y": 7,
    "z": 7,
    "intensity": 7,
    "ring": 4,  # PointField.UINT16
    "timestamp": 8,  # PointField.FLOAT64, seconds
}
STRUCT_FORMATS = {
    1: "b",
    2: "B",
    3: "h",
    4: "H",
    5: "i",
    6: "I",
    7: "f",
    8: "d",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check PandarXT PointCloud2 fields and per-point timestamps."
    )
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--expected-frame", default="hesai_lidar")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--min-scan-span", type=float, default=0.05)
    parser.add_argument("--max-scan-span", type=float, default=0.15)
    parser.add_argument("--header-tolerance", type=float, default=0.02)
    return parser.parse_args()


def unpack_field(message, field):
    if field.datatype not in STRUCT_FORMATS or field.count != 1:
        raise ValueError(
            f"unsupported timestamp layout: datatype={field.datatype}, "
            f"count={field.count}"
        )

    byte_order = ">" if message.is_bigendian else "<"
    unpacker = struct.Struct(byte_order + STRUCT_FORMATS[field.datatype])
    values = []
    for row in range(message.height):
        row_offset = row * message.row_step
        for column in range(message.width):
            offset = row_offset + column * message.point_step + field.offset
            values.append(unpacker.unpack_from(message.data, offset)[0])
    return values


def main():
    args = parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")
    if not 0 <= args.min_scan_span < args.max_scan_span:
        raise SystemExit("scan-span bounds are invalid")

    try:
        import rclpy
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        print(f"FAIL — ROS 2 Python dependency unavailable: {exc}", file=sys.stderr)
        return 1

    rclpy.init(args=None)
    node = rclpy.create_node("check_pandarxt_pointcloud")
    received = []
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(
        PointCloud2, args.topic, lambda message: received.append(message), qos
    )

    deadline = time.monotonic() + args.timeout
    try:
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    if not received:
        print(f"{PENDING}: no PointCloud2 message received on {args.topic}")
        return 2

    message = received[0]
    failures = []
    warnings = []
    fields = {field.name: field for field in message.fields}

    for name, datatype in EXPECTED_FIELDS.items():
        field = fields.get(name)
        if field is None:
            failures.append(f"missing field {name!r}")
        elif field.datatype != datatype or field.count != 1:
            failures.append(
                f"field {name!r} has datatype/count "
                f"{field.datatype}/{field.count}, expected {datatype}/1"
            )

    if message.header.frame_id != args.expected_frame:
        failures.append(
            f"frame_id={message.header.frame_id!r}, expected {args.expected_frame!r}"
        )

    timestamp_field = fields.get("timestamp")
    timestamps = []
    if timestamp_field is not None:
        try:
            timestamps = [float(value) for value in unpack_field(message, timestamp_field)]
        except (ValueError, struct.error) as exc:
            failures.append(str(exc))

    finite_timestamps = [value for value in timestamps if math.isfinite(value)]
    if len(finite_timestamps) != len(timestamps):
        failures.append(
            f"{len(timestamps) - len(finite_timestamps)} timestamp values are NaN/Inf"
        )

    scan_span = math.nan
    unique_count = 0
    if finite_timestamps:
        minimum = min(finite_timestamps)
        maximum = max(finite_timestamps)
        scan_span = maximum - minimum
        unique_count = len(set(finite_timestamps))

        if unique_count < 2:
            failures.append("timestamp is constant across the cloud")
        if not args.min_scan_span <= scan_span <= args.max_scan_span:
            failures.append(
                f"timestamp span {scan_span:.9f} s is outside "
                f"[{args.min_scan_span}, {args.max_scan_span}] s"
            )

        if unique_count != len(finite_timestamps):
            warnings.append(
                "timestamps repeat within the scan; the current Hesai source "
                "exports packet-level time rather than a unique time for every point"
            )

        if abs(minimum) > 5.0:
            timestamp_convention = "absolute seconds"
            header_time = (
                float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) * 1.0e-9
            )
            header_delta = abs(minimum - header_time)
            if header_delta > args.header_tolerance:
                failures.append(
                    f"first point/header time delta {header_delta:.9f} s exceeds "
                    f"{args.header_tolerance} s"
                )
        else:
            timestamp_convention = "relative seconds"
            header_delta = math.nan
    else:
        timestamp_convention = "unavailable"
        header_delta = math.nan
        failures.append("no finite timestamp values were decoded")

    point_count = message.width * message.height
    print(f"topic: {args.topic}")
    print(f"frame_id: {message.header.frame_id}")
    print(f"points: {point_count}")
    print(f"point_step: {message.point_step}")
    print("fields: " + ", ".join(field.name for field in message.fields))
    print(f"timestamp convention: {timestamp_convention}")
    print(f"timestamp unique values: {unique_count}/{len(timestamps)}")
    print(f"timestamp scan span: {scan_span:.9f} s")
    if math.isfinite(header_delta):
        print(f"first point/header delta: {header_delta:.9f} s")

    for warning in warnings:
        print(f"WARN — {warning}")
    for failure in failures:
        print(f"FAIL — {failure}", file=sys.stderr)

    if failures:
        return 1
    print("PASS — PandarXT PointCloud2 schema and timestamp range are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
