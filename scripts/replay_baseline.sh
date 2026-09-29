#!/usr/bin/env bash
#
# Replay a raw baseline bag (recorded by record_baseline_bag.sh) through MOLA.
#
#   map       Build a map from the bag (mapping mode), save it under <map-prefix>.
#   localize  Replay the same bag against <map-prefix> in localization-only mode,
#             record MOLA outputs, and print the final /diagnostics numbers
#             (avg_ms, utilization_pct, dropped_frames_ratio, quality).
#
# Typical C-3 sequence:
#   ./scripts/replay_baseline.sh map      <bag-dir> ~/a1_maps/<date>/map
#   ./scripts/replay_baseline.sh localize <bag-dir> ~/a1_maps/<date>/map            # timing run, rviz off
#   ./scripts/replay_baseline.sh localize <bag-dir> ~/a1_maps/<date>/map --rviz     # timing run, rviz on
#   ./scripts/replay_baseline.sh localize <bag-dir> ~/a1_maps/<date>/map --icp-logs # quality run
#
# Timing numbers are only meaningful on the vehicle PC. Quality numbers
# (--icp-logs) do not depend on the machine.

set -euo pipefail

readonly PKG="a1_mola_localization"
readonly PLAY_TOPICS=(/lidar_points /tf /tf_static)
readonly RECORD_TOPICS=(
  /diagnostics
  /lidar_odometry/pose
  /lidar_odometry/pose_quality
  /mola_diagnostics/lidar_odom/status
)
readonly READY_TIMEOUT_S="${READY_TIMEOUT_S:-90}"
readonly ROS_COMMAND_TIMEOUT="${ROS_COMMAND_TIMEOUT:-15s}"

usage() {
  cat >&2 <<EOF
Usage:
  $0 map      <bag-dir> <map-prefix> [--rate <x>]
  $0 localize <bag-dir> <map-prefix> [--rate <x>] [--rviz] [--icp-logs] [--out <dir>]

  bag-dir      rosbag2 directory containing metadata.yaml
  map-prefix   Map path without extension (.mm/.simplemap are added)
  --rate       Playback rate (default 1.0; keep 1.0 for timing runs)
  --rviz       Start RViz (default off; timing differs with it on)
  --icp-logs   Set MP2P_ICP_GENERATE_DEBUG_FILES=1 and MOLA_LO_DEBUG_ICP_QUALITY=true.
               Writes icp-logs/ and a per-frame quality CSV into --out.
               Do not use this for timing runs.
  --out        Output directory for recorded bag/logs
               (default: <bag-dir>/../replay_<mode>_<timestamp>)

Source ROS 2 Humble and this workspace before running. Do not run the
LiDAR driver at the same time; the bag replaces it.
EOF
}

info() { printf '[INFO] %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1" >&2; }
fail() { printf '[FAIL] %s\n' "$1" >&2; exit 1; }

# ---------------------------------------------------------------- arguments
[[ $# -ge 3 ]] || { usage; exit 1; }
mode="$1"; bag_dir="$2"; map_prefix="$3"; shift 3

rate="1.0"; rviz="false"; icp_logs="false"; out_dir=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate) rate="$2"; shift 2 ;;
    --rviz) rviz="true"; shift ;;
    --icp-logs) icp_logs="true"; shift ;;
    --out) out_dir="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; fail "Unknown argument: $1" ;;
  esac
done

case "${mode}" in
  map|localize) ;;
  *) usage; fail "mode must be 'map' or 'localize'" ;;
esac

bag_dir="$(cd "${bag_dir}" 2>/dev/null && pwd)" || fail "Bag directory not found"
[[ -f "${bag_dir}/metadata.yaml" ]] || fail "Not a rosbag2 directory (no metadata.yaml): ${bag_dir}"
grep -q "name: /lidar_points" "${bag_dir}/metadata.yaml" \
  || fail "Bag does not contain /lidar_points: ${bag_dir}"

map_prefix="${map_prefix%.mm}"; map_prefix="${map_prefix%.simplemap}"
if [[ "${mode}" == "localize" ]]; then
  [[ -f "${map_prefix}.mm" ]] || fail "Map not found: ${map_prefix}.mm (run 'map' first)"
else
  [[ -e "${map_prefix}.mm" ]] && fail "Map already exists, refusing to overwrite: ${map_prefix}.mm"
  mkdir -p "$(dirname "${map_prefix}")"
fi

if [[ -z "${out_dir}" ]]; then
  out_dir="$(dirname "${bag_dir}")/replay_${mode}_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${out_dir}"
launch_log="${out_dir}/launch.log"

command -v ros2 >/dev/null 2>&1 || fail "ros2 not found. Source ROS 2 Humble first."
ros2 pkg prefix "${PKG}" >/dev/null 2>&1 || fail "${PKG} not found. Source install/setup.zsh first."

if timeout "${ROS_COMMAND_TIMEOUT}" ros2 topic list 2>/dev/null | grep -qx "/lidar_points"; then
  fail "/lidar_points is already being published (live driver?). Stop it before replaying."
fi

# ---------------------------------------------------------------- environment
if [[ "${icp_logs}" == "true" ]]; then
  export MP2P_ICP_GENERATE_DEBUG_FILES=1
  export MP2P_ICP_LOG_FILES_DECIMATION="${MP2P_ICP_LOG_FILES_DECIMATION:-10}"
  export MOLA_LO_DEBUG_ICP_QUALITY=true
  warn "ICP debug files enabled: timing from this run is NOT a valid baseline."
fi

launch_args=(
  "start_lidar_driver:=false"
  "use_sim_time:=true"
  "rviz:=${rviz}"
  "mola_gui:=false"
)
if [[ "${mode}" == "map" ]]; then
  launch_file="mapping.launch.py"
  launch_args+=("map_output_path:=${map_prefix}")
  ready_service="/map_save"
else
  launch_file="localization.launch.py"
  launch_args+=("map:=${map_prefix}")
  ready_service="/relocalize_near_pose"
fi

# ---------------------------------------------------------------- cleanup
launch_pid=""; record_pid=""
cleanup() {
  set +e
  if [[ -n "${record_pid}" ]] && kill -0 "${record_pid}" 2>/dev/null; then
    kill -INT "${record_pid}"; wait "${record_pid}" 2>/dev/null
  fi
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}"
    # MOLA saves the map / simplemap on shutdown in mapping mode; give it time.
    for _ in $(seq 1 60); do kill -0 "${launch_pid}" 2>/dev/null || break; sleep 1; done
    kill -0 "${launch_pid}" 2>/dev/null && kill -TERM "${launch_pid}"
    wait "${launch_pid}" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------- launch MOLA
info "Mode: ${mode}   bag: ${bag_dir}"
info "Map prefix: ${map_prefix}"
info "Outputs: ${out_dir}"
info "Launching ${PKG} ${launch_file} ${launch_args[*]}"

# icp-logs/ is written relative to the working directory.
cd "${out_dir}"
ros2 launch "${PKG}" "${launch_file}" "${launch_args[@]}" >"${launch_log}" 2>&1 &
launch_pid=$!

info "Waiting for ${ready_service} (up to ${READY_TIMEOUT_S}s)..."
ready="false"
for _ in $(seq 1 "${READY_TIMEOUT_S}"); do
  kill -0 "${launch_pid}" 2>/dev/null || fail "Launch exited early. See ${launch_log}"
  if timeout 5s ros2 service list 2>/dev/null | grep -qx "${ready_service}"; then
    ready="true"; break
  fi
  sleep 1
done
[[ "${ready}" == "true" ]] || fail "MOLA did not become ready. See ${launch_log}"
info "MOLA ready."

# ---------------------------------------------------------------- localize: seed + record
if [[ "${mode}" == "localize" ]]; then
  # The bag starts where the map's origin is (same bag built the map), so an
  # identity pose in 'map' is the correct seed. The local relocalization patch
  # activates MOLA automatically after a pose-based request.
  info "Seeding initial pose at map origin via ${ready_service}..."
  timeout 20s ros2 service call /relocalize_near_pose mola_msgs/srv/RelocalizeNearPose \
    "{pose: {header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}}" \
    | grep -q "accepted=True" || fail "relocalize_near_pose was not accepted"

  info "Recording MOLA outputs to ${out_dir}/outputs"
  ros2 bag record -s mcap -o "${out_dir}/outputs" "${RECORD_TOPICS[@]}" \
    >"${out_dir}/record.log" 2>&1 &
  record_pid=$!
  sleep 2
fi

# ---------------------------------------------------------------- play
info "Playing bag at rate ${rate} (topics: ${PLAY_TOPICS[*]})..."
ros2 bag play "${bag_dir}" --clock --rate "${rate}" --topics "${PLAY_TOPICS[@]}" \
  >"${out_dir}/play.log" 2>&1
info "Playback finished. Letting MOLA drain for 5s..."
sleep 5

# ---------------------------------------------------------------- results
if [[ "${mode}" == "map" ]]; then
  info "Saving map via /map_save..."
  timeout 120s ros2 service call /map_save mola_msgs/srv/MapSave \
    "{map_path: '${map_prefix}'}" >>"${launch_log}" 2>&1 || warn "/map_save call failed; shutdown save will still be attempted"
  cleanup; trap - EXIT
  printf '\n==== Map ====\n'
  ls -la "${map_prefix}".mm "${map_prefix}".simplemap 2>/dev/null || fail "Map files were not written"
  command -v sm-cli >/dev/null 2>&1 && sm-cli info "${map_prefix}.simplemap" | grep -E "keyframe_count|timestamp_span|label" || true
  exit 0
fi

printf '\n==== Final /diagnostics ====\n'
diag="$(timeout 15s ros2 topic echo /diagnostics --once 2>/dev/null || true)"
if [[ -z "${diag}" ]]; then
  warn "No /diagnostics message received; inspect ${out_dir}/outputs instead."
else
  # Print key/value pairs for the LidarOdometry status blocks.
  awk '
    /^- level:/ { name = "" }
    /^  name: / { sub(/^  name: /, ""); gsub(/\047/, ""); name = $0 }
    /^  - key: / { key = $3 }
    /^    value: / { sub(/^    value: /, ""); gsub(/\047/, "");
                     if (name ~ /LidarOdometry/) printf "%-32s %-24s %s\n", name, key, $0 }
  ' <<<"${diag}" | tee "${out_dir}/final_diagnostics.txt"
fi

if [[ "${icp_logs}" == "true" ]]; then
  csv="${out_dir}/icp_quality.csv"
  printf 'pathStep,timestamp,goodness,minRequired,iters,termReason,isGood,adapt_thres_sigma,consecutive_bad\n' >"${csv}"
  grep '^\[LidarOdometry\] pathStep=' "${launch_log}" \
    | sed -E 's/^\[LidarOdometry\] //; s#([0-9]{4}/[0-9]{2}/[0-9]{2}),#\1_#; s/[A-Za-z_]+=//g; s/ +/,/g' >>"${csv}" || true
  n=$(( $(wc -l <"${csv}") - 1 ))
  printf '\n==== ICP quality (%d frames) -> %s ====\n' "${n}" "${csv}"
  if [[ "${n}" -gt 0 ]]; then
    median="$(tail -n +2 "${csv}" | cut -d, -f3 | sort -g | awk '{a[NR]=$1} END {print a[int((NR+1)/2)]}')"
    tail -n +2 "${csv}" | awk -F, -v n="${n}" -v med="${median}" '
      { if ($3 < 0.5) bad++; if ($5 >= 25) maxit++ }
      END { printf "goodness median : %s\n", med;
            printf "goodness < 0.5  : %.1f%%\n", 100*bad/n;
            printf "iters == max(25): %.1f%%\n", 100*maxit/n }'
  fi
  info "ICP debug files: ${out_dir}/icp-logs/"
fi

info "Done. Launch log: ${launch_log}"
