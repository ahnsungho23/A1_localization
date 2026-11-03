#!/usr/bin/env bash

set -euo pipefail

readonly DEFAULT_LIDAR_IP="192.168.1.201"
readonly REQUIRED_RMEM_MAX=33554432
readonly POINTCLOUD_TYPE="sensor_msgs/msg/PointCloud2"
readonly PACKET_LOSS_TYPE="hesai_ros_driver/msg/LossPacket"
readonly EXPECTED_FRAME_ID="hesai_lidar"
readonly EXPECTED_POINTCLOUD_WIDTH=128000
readonly EXPECTED_POINT_STEP=26
readonly ROS_COMMAND_TIMEOUT="${ROS_COMMAND_TIMEOUT:-15s}"
readonly ROS_RETRY_ATTEMPTS=3
readonly ROS_RETRY_DELAY_SECONDS=1

usage() {
  cat >&2 <<EOF
Usage: $0 <ethernet-interface> [lidar-ip]

Run read-only PandarXT host and ROS checks.
  ethernet-interface  Required interface connected to the LiDAR
  lidar-ip            Optional LiDAR address (default: ${DEFAULT_LIDAR_IP})

Source ROS 2 Humble and this workspace before running the script.
EOF
}

section() {
  printf '\n==== %s ====\n' "$1"
}

pass() {
  printf '[PASS] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1" >&2
}

fail() {
  printf '[FAIL] %s\n' "$1" >&2
  failures=$((failures + 1))
}

validate_ipv4_address() {
  local address="$1"
  local octet
  local -a octets

  IFS='.' read -r -a octets <<< "${address}"
  [[ "${#octets[@]}" -eq 4 ]] || return 1

  for octet in "${octets[@]}"; do
    [[ "${octet}" =~ ^[0-9]+$ ]] || return 1
    ((10#${octet} <= 255)) || return 1
  done
}

run_ros_with_retry() {
  local description="$1"
  shift

  local attempt command_status
  local command_output=""

  last_ros_output=""
  for ((attempt = 1; attempt <= ROS_RETRY_ATTEMPTS; attempt++)); do
    if command_output="$(timeout "${ROS_COMMAND_TIMEOUT}" "$@" 2>&1)"; then
      last_ros_output="${command_output}"
      return 0
    else
      command_status=$?
    fi

    last_ros_output="${command_output}"
    warn "${description}: attempt ${attempt}/${ROS_RETRY_ATTEMPTS} failed (status ${command_status})."
    if ((attempt < ROS_RETRY_ATTEMPTS)); then
      sleep "${ROS_RETRY_DELAY_SECONDS}"
    fi
  done

  return 1
}

report_cli_loss_warning() {
  local command_output="$1"

  if [[ "${command_output}" == *"A message was lost"* ]]; then
    warn "ros2 topic echo reported a lost CLI subscription sample; this is not LiDAR UDP packet-loss evidence."
  fi
}

if (($# < 1 || $# > 2)); then
  usage
  exit 2
fi

readonly INTERFACE="$1"
readonly LIDAR_IP="${2:-${DEFAULT_LIDAR_IP}}"

missing_command=0
for command_name in ip sysctl ping timeout ros2 awk sleep python3; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: Required command not found: ${command_name}" >&2
    missing_command=1
  fi
done
if ((missing_command != 0)); then
  exit 1
fi

if ! ip link show dev "${INTERFACE}" >/dev/null 2>&1; then
  echo "ERROR: Interface does not exist: ${INTERFACE}" >&2
  exit 1
fi

if ! validate_ipv4_address "${LIDAR_IP}"; then
  echo "ERROR: Invalid LiDAR IPv4 address: ${LIDAR_IP}" >&2
  exit 1
fi

failures=0
last_ros_output=""
pointcloud_received=0
topic_info_usable=0
topic_info_type=""
publisher_count=""

section "Interface link and address"
ip -brief link show dev "${INTERFACE}"
ip -brief address show dev "${INTERFACE}"
pass "Interface exists and its read-only state was displayed."

section "Host UDP receive buffer"
rmem_max="$(sysctl -n net.core.rmem_max)"
printf 'net.core.rmem_max = %s\n' "${rmem_max}"
if [[ "${rmem_max}" =~ ^[0-9]+$ ]] && ((rmem_max >= REQUIRED_RMEM_MAX)); then
  pass "Receive-buffer maximum is at least ${REQUIRED_RMEM_MAX}."
else
  fail "Receive-buffer maximum is below ${REQUIRED_RMEM_MAX}."
fi

section "LiDAR reachability"
if timeout 8s ping -I "${INTERFACE}" -c 3 -W 1 "${LIDAR_IP}"; then
  pass "LiDAR ${LIDAR_IP} replied on ${INTERFACE}."
else
  fail "LiDAR ${LIDAR_IP} did not reply on ${INTERFACE}."
fi

section "ROS topic and type discovery"
if run_ros_with_retry "ROS topic/type discovery" \
    ros2 topic list --no-daemon --spin-time 3 -t; then
  topic_list_output="${last_ros_output}"
  printf '%s\n' "${topic_list_output}"
  if [[ "${topic_list_output}" == *"/lidar_points"* ]]; then
    pass "Discovery reported /lidar_points."
  else
    warn "Discovery did not list /lidar_points; explicit-type message reception will still be attempted."
  fi
else
  printf '%s\n' "${last_ros_output}" >&2
  warn "Topic/type discovery remained unavailable; explicit-type message reception will still be attempted."
fi

section "/lidar_points topic information"
if run_ros_with_retry "/lidar_points topic info" \
    ros2 topic info /lidar_points --no-daemon --spin-time 3; then
  topic_info_output="${last_ros_output}"
  printf '%s\n' "${topic_info_output}"
  topic_info_type="$(awk -F ': *' '/^Type:/ {print $2; exit}' \
    <<< "${topic_info_output}")"
  publisher_count="$(awk -F ': *' '/^Publisher count:/ {print $2; exit}' \
    <<< "${topic_info_output}")"
  if [[ -n "${topic_info_type}" && -n "${publisher_count}" ]]; then
    topic_info_usable=1
  else
    warn "Topic info returned without a usable type or publisher count."
  fi
else
  printf '%s\n' "${last_ros_output}" >&2
  warn "Topic info remained unavailable after retries."
fi

section "PointCloud2 message, header, width, and point_step"
if run_ros_with_retry "PointCloud2 message reception" \
    ros2 topic echo /lidar_points "${POINTCLOUD_TYPE}" \
    --no-daemon --spin-time 3 \
    --qos-reliability best_effort --qos-depth 1 \
    --once --no-arr; then
  pointcloud_output="${last_ros_output}"
  printf '%s\n' "${pointcloud_output}"
  report_cli_loss_warning "${pointcloud_output}"
  pointcloud_received=1
  pass "Received an actual PointCloud2 message with an explicit type."

  header_sec="$(awk '$1 == "sec:" {print $2; exit}' \
    <<< "${pointcloud_output}")"
  header_nanosec="$(awk '$1 == "nanosec:" {print $2; exit}' \
    <<< "${pointcloud_output}")"
  frame_id="$(awk '$1 == "frame_id:" {print $2; exit}' \
    <<< "${pointcloud_output}")"
  width="$(awk '$1 == "width:" {print $2; exit}' \
    <<< "${pointcloud_output}")"
  point_step="$(awk '$1 == "point_step:" {print $2; exit}' \
    <<< "${pointcloud_output}")"

  if [[ "${header_sec}" =~ ^[0-9]+$ && \
        "${header_nanosec}" =~ ^[0-9]+$ && \
        "${frame_id}" == "${EXPECTED_FRAME_ID}" ]]; then
    pass "Header stamp is present and frame_id is ${EXPECTED_FRAME_ID}."
  else
    fail "PointCloud2 header is invalid (sec='${header_sec:-missing}', nanosec='${header_nanosec:-missing}', frame_id='${frame_id:-missing}')."
  fi

  if [[ "${width}" == "${EXPECTED_POINTCLOUD_WIDTH}" ]]; then
    pass "PointCloud2 width is ${EXPECTED_POINTCLOUD_WIDTH}."
  else
    fail "PointCloud2 width is '${width:-missing}', expected ${EXPECTED_POINTCLOUD_WIDTH}."
  fi

  if [[ "${point_step}" == "${EXPECTED_POINT_STEP}" ]]; then
    pass "PointCloud2 point_step is ${EXPECTED_POINT_STEP}."
  else
    fail "PointCloud2 point_step is '${point_step:-missing}', expected ${EXPECTED_POINT_STEP}."
  fi
else
  printf '%s\n' "${last_ros_output}" >&2
  report_cli_loss_warning "${last_ros_output}"
  fail "No PointCloud2 message was received after retries."
fi

section "PointCloud2 fields via rclpy"
field_check_status=0
if python3 - <<'PY'
import sys
import time

TIMEOUT_SECONDS = 15.0
EXPECTED_FIELDS = [
    ("x", 0, 7, 1),
    ("y", 4, 7, 1),
    ("z", 8, 7, 1),
    ("intensity", 12, 7, 1),
    ("ring", 16, 4, 1),
    ("timestamp", 18, 8, 1),
]


def format_fields(fields):
    if fields is None:
        return "  <no message received>"
    if not fields:
        return "  <empty>"
    return "\n".join(
        "  {index}: name={name!r}, offset={offset}, "
        "datatype={datatype}, count={count}".format(
            index=index,
            name=name,
            offset=offset,
            datatype=datatype,
            count=count,
        )
        for index, (name, offset, datatype, count) in enumerate(fields)
    )


try:
    import rclpy
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2
except Exception as exc:
    print("Expected fields:")
    print(format_fields(EXPECTED_FIELDS))
    print("Actual fields:")
    print("  <unavailable>")
    print(f"ERROR: ROS Python imports failed: {exc}", file=sys.stderr)
    raise SystemExit(3)


def receive_fields():
    received_fields = None
    node = None

    rclpy.init(args=None)
    try:
        node = rclpy.create_node("a1_lidar_field_verifier")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def callback(msg):
            nonlocal received_fields
            received_fields = [
                (field.name, field.offset, field.datatype, field.count)
                for field in msg.fields
            ]

        subscription = node.create_subscription(
            PointCloud2,
            "/lidar_points",
            callback,
            qos,
        )
        deadline = time.monotonic() + TIMEOUT_SECONDS
        while received_fields is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.25, remaining))

        # Keep the subscription alive until the receive loop has finished.
        del subscription
        return received_fields
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    print("Expected fields:")
    print(format_fields(EXPECTED_FIELDS))

    try:
        actual_fields = receive_fields()
    except Exception as exc:  # Report ROS initialization/subscription failures clearly.
        print("Actual fields:")
        print("  <unavailable>")
        print(f"ERROR: rclpy field inspection failed: {exc}", file=sys.stderr)
        return 3

    print("Actual fields:")
    print(format_fields(actual_fields))

    if actual_fields is None:
        print(
            "ERROR: Timed out after 15 seconds waiting for a "
            "PointCloud2 message on /lidar_points.",
            file=sys.stderr,
        )
        return 2

    mismatches = []
    if len(actual_fields) != len(EXPECTED_FIELDS):
        mismatches.append(
            f"field count: expected {len(EXPECTED_FIELDS)}, "
            f"actual {len(actual_fields)}"
        )

    attribute_names = ("name", "offset", "datatype", "count")
    for index in range(max(len(EXPECTED_FIELDS), len(actual_fields))):
        if index >= len(EXPECTED_FIELDS):
            mismatches.append(
                f"index {index}: unexpected field {actual_fields[index]!r}"
            )
            continue
        if index >= len(actual_fields):
            mismatches.append(
                f"index {index}: missing expected field {EXPECTED_FIELDS[index]!r}"
            )
            continue

        expected = EXPECTED_FIELDS[index]
        actual = actual_fields[index]
        for attribute_index, attribute_name in enumerate(attribute_names):
            if expected[attribute_index] != actual[attribute_index]:
                mismatches.append(
                    f"index {index} {attribute_name}: "
                    f"expected {expected[attribute_index]!r}, "
                    f"actual {actual[attribute_index]!r}"
                )

    if mismatches:
        print("Field mismatches:", file=sys.stderr)
        for mismatch in mismatches:
            print(f"  - {mismatch}", file=sys.stderr)
        return 1

    return 0


raise SystemExit(main())
PY
then
  pointcloud_received=1
  pass "PointCloud2 fields match in order, name, offset, datatype, and count."
else
  field_check_status=$?
  case "${field_check_status}" in
    1)
      pointcloud_received=1
      fail "PointCloud2 field schema does not match the expected PandarXT schema."
      ;;
    2)
      fail "rclpy field inspection timed out after 15 seconds without a PointCloud2 message."
      ;;
    *)
      fail "rclpy field inspection failed with status ${field_check_status}."
      ;;
  esac
fi

section "/lidar_points publisher verdict"
if ((topic_info_usable != 0)); then
  if [[ "${topic_info_type}" == "${POINTCLOUD_TYPE}" ]]; then
    pass "/lidar_points type is ${POINTCLOUD_TYPE}."
  else
    fail "/lidar_points type is '${topic_info_type}', expected ${POINTCLOUD_TYPE}."
  fi

  if [[ "${publisher_count}" == "1" ]]; then
    pass "/lidar_points has exactly one publisher."
  else
    fail "/lidar_points publisher count is '${publisher_count}', expected 1."
  fi
elif ((pointcloud_received != 0)); then
  warn "Publisher/type discovery was unavailable, but an explicit-type PointCloud2 message was received; publisher inspection is not a final failure."
else
  fail "Publisher/type discovery was unavailable and no PointCloud2 message was received."
fi

section "/lidar_packets_loss"
if run_ros_with_retry "/lidar_packets_loss reception" \
    ros2 topic echo /lidar_packets_loss "${PACKET_LOSS_TYPE}" \
    --no-daemon --spin-time 3 \
    --qos-reliability best_effort --qos-depth 1 --once; then
  loss_output="${last_ros_output}"
  printf '%s\n' "${loss_output}"
  report_cli_loss_warning "${loss_output}"
  loss_count="$(awk -F ': *' '/^total_packet_loss_count:/ {print $2; exit}' \
    <<< "${loss_output}")"
  if [[ "${loss_count}" == "0" ]]; then
    pass "LiDAR driver reports zero packet loss."
  else
    fail "LiDAR packet loss is '${loss_count:-missing}', expected 0."
  fi
else
  printf '%s\n' "${last_ros_output}" >&2
  report_cli_loss_warning "${last_ros_output}"
  fail "Could not read /lidar_packets_loss after retries."
fi

section "Kernel UDP error counters"
if udp_errors="$(awk '
  $1 == "Udp:" && !seen_header {
    for (i = 2; i <= NF; i++) header[i] = $i
    seen_header = 1
    next
  }
  $1 == "Udp:" && seen_header {
    for (i = 2; i <= NF; i++) {
      if (header[i] == "InErrors" || header[i] == "RcvbufErrors") {
        printf "Udp%s=%s\n", header[i], $i
      }
    }
    found_values = 1
    exit
  }
  END { if (!found_values) exit 1 }
' /proc/net/snmp)"; then
  printf '%s\n' "${udp_errors}"
  warn "These counters are cumulative and host-wide; compare before/after values when diagnosing a run."
else
  fail "Could not read UdpInErrors and UdpRcvbufErrors from /proc/net/snmp."
fi

section "Result"
echo "Actual explicit-type PointCloud2 reception is the primary ROS success condition."
echo "A ros2 CLI 'A message was lost' warning is not treated as LiDAR UDP packet loss."
echo "ros2 topic hz and ros2 param get are not used as success criteria."
echo "This script never restarts the ROS daemon and does not change network settings."
if ((failures == 0)); then
  pass "All required read-only checks completed successfully."
  exit 0
fi

printf '[FAIL] %d required check(s) failed.\n' "${failures}" >&2
exit 1
