#!/usr/bin/env bash
#
# Record the raw sensor topics needed for an offline baseline (C-2 / C-5).
#
# The base topic list is fixed. Applanix GNSS/IMU topics are not known yet;
# append them as extra arguments once the driver is confirmed, e.g.
#   ./scripts/record_baseline_bag.sh /applanix/navsatfix /applanix/imu
#
# Do NOT record MOLA outputs here. The point of this bag is to be replayable
# against any future MOLA configuration, so only raw inputs belong in it.

set -euo pipefail

readonly BASE_TOPICS=(
  /lidar_points
  /lidar_packets_loss
  /lidar_ptp
  /tf
  /tf_static
)
readonly DEFAULT_OUT_ROOT="${HOME}/.ros/a1_localization/bags"
readonly ROS_COMMAND_TIMEOUT="${ROS_COMMAND_TIMEOUT:-10s}"

usage() {
  cat >&2 <<EOF
Usage: $0 [-o <output-dir>] [extra-topic ...]

Records raw sensor topics into an mcap rosbag2 directory.
  -o <output-dir>   Bag directory to create
                    (default: ${DEFAULT_OUT_ROOT}/a1_baseline_<YYYYmmdd_HHMMSS>)
  extra-topic       Additional topics, e.g. Applanix GNSS/IMU once known

Base topics always recorded:
$(printf '  %s\n' "${BASE_TOPICS[@]}")

Source ROS 2 Humble and this workspace, and have the LiDAR driver running,
before starting. Stop with Ctrl-C; the bag is finalized on exit.
EOF
}

info() { printf '[INFO] %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1" >&2; }
fail() { printf '[FAIL] %s\n' "$1" >&2; exit 1; }

out_dir=""
while getopts ":o:h" opt; do
  case "${opt}" in
    o) out_dir="${OPTARG}" ;;
    h) usage; exit 0 ;;
    \?) usage; fail "Unknown option: -${OPTARG}" ;;
    :) usage; fail "Option -${OPTARG} requires a value" ;;
  esac
done
shift $((OPTIND - 1))
extra_topics=("$@")

if [[ -z "${out_dir}" ]]; then
  out_dir="${DEFAULT_OUT_ROOT}/a1_baseline_$(date +%Y%m%d_%H%M%S)"
fi
if [[ -e "${out_dir}" ]]; then
  fail "Output already exists, refusing to overwrite: ${out_dir}"
fi

command -v ros2 >/dev/null 2>&1 || fail "ros2 not found. Source ROS 2 Humble first."

info "Checking live topics (timeout ${ROS_COMMAND_TIMEOUT})..."
live_topics="$(timeout "${ROS_COMMAND_TIMEOUT}" ros2 topic list 2>/dev/null || true)"

if ! grep -qx "/lidar_points" <<<"${live_topics}"; then
  fail "/lidar_points is not being published. Start the PandarXT driver first."
fi
if ! grep -qx "/lidar_ptp" <<<"${live_topics}"; then
  warn "/lidar_ptp is not published. Enable ros_send_ptp_topic in" \
       "src/a1_lidar_bringup/config/pandarxt.yaml.in if PTP lock status is wanted."
fi
for t in "${extra_topics[@]}"; do
  if ! grep -qx "${t}" <<<"${live_topics}"; then
    warn "Extra topic ${t} is not currently published; it will be recorded if it appears."
  fi
done

all_topics=("${BASE_TOPICS[@]}" "${extra_topics[@]}")
mkdir -p "$(dirname "${out_dir}")"

info "Recording to ${out_dir}"
printf '  %s\n' "${all_topics[@]}"
info "Press Ctrl-C to stop."

# ros2 bag handles SIGINT and finalizes the bag; print a summary afterwards.
set +e
ros2 bag record -s mcap -o "${out_dir}" "${all_topics[@]}"
rc=$?
set -e

if [[ -f "${out_dir}/metadata.yaml" ]]; then
  printf '\n==== Bag summary ====\n'
  ros2 bag info "${out_dir}" || true
  info "Check that /lidar_points count is close to (duration_s x 10)."
else
  fail "No bag was written (exit code ${rc})."
fi
