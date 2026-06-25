#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_glim_online_reliable.sh --bag <rosbag_dir> --pcap <ins_pcap> --output <run_dir> [options]

Options:
  --rate <rate>          ros2 bag replay rate (default: 0.2)
  --domain <id>          ROS_DOMAIN_ID (default: 86)
  --duration <seconds>   Optional ros2 bag --playback-duration
  --viewer <true|false>  Enable GLIM standard/RViz viewer modules (default: true)
  --no-viewer            Same as --viewer false

This is an online GLIM mapping runner for raw Putnam-style data. It does not
run prep_bag or create a preprocessed bag. The raw rosbag and INS PCAP are
played live:

  raw rosbag /atlas/pose_filtered
    + /luminar_front/points
    + /luminar_left/points
    + /luminar_right/points
    + INS PCAP
    -> adapter publishes /gps_p1/*
    -> GLIM consumes /gps_p1/* and 3-LiDAR concat from /luminar_front/points

The runner creates a run-local GLIM config under <run_dir>/config and forces
all three /luminar_* pointcloud replay/subscriptions to reliable QoS. This
keeps the online 3-LiDAR stream deterministic enough for GLIM's lidar_concat
path and avoids the best-effort LiDAR drops that caused online LiDAR gaps and
u-turn failures.
EOF
}

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_dir=""
pcap_path=""
output_dir=""
rate="0.2"
domain_id="86"
duration=""
viewer="true"
post_bag_drain_sec="${GLIM_POST_BAG_DRAIN_SEC:-60}"
tail_wait_stall_sec="${GLIM_TAIL_WAIT_STALL_SEC:-45}"
glim_save_timeout_sec="${GLIM_SAVE_TIMEOUT_SEC:-1800}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --bag)
      bag_dir="$2"; shift 2;;
    --pcap)
      pcap_path="$2"; shift 2;;
    --output)
      output_dir="$2"; shift 2;;
    --rate)
      rate="$2"; shift 2;;
    --domain)
      domain_id="$2"; shift 2;;
    --duration)
      duration="$2"; shift 2;;
    --viewer)
      viewer="$2"; shift 2;;
    --no-viewer)
      viewer="false"; shift;;
    -h|--help)
      usage; exit 0;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2;;
  esac
done

if [ -z "$bag_dir" ] || [ -z "$pcap_path" ] || [ -z "$output_dir" ]; then
  usage >&2
  exit 2
fi

mkdir -p "$output_dir" "$output_dir/ros_log" "$output_dir/ros_home" "$output_dir/dump"

config_dir="$output_dir/config"
rm -rf "$config_dir"
cp -a "$repo_root/GLIM/glim/config" "$config_dir"
if [ -f "$repo_root/GLIM/glim_ext/config/config_gnss_global.json" ]; then
  cp "$repo_root/GLIM/glim_ext/config/config_gnss_global.json" "$config_dir/"
fi

python3 - "$config_dir" "$viewer" <<'PY'
from pathlib import Path
import re
import sys

config_dir = Path(sys.argv[1])
viewer = sys.argv[2].lower() in {"1", "true", "yes", "on"}

def replace(path, pattern, repl):
    text = path.read_text()
    new_text, n = re.subn(pattern, repl, text, flags=re.S)
    if n == 0:
        raise SystemExit(f"pattern not found in {path}: {pattern}")
    path.write_text(new_text)

config_json = config_dir / "config.json"
replace(config_json, r'"config_odometry"\s*:\s*"[^"]+"', '"config_odometry": "config_odometry_ins.json"')
replace(config_json, r'"config_sub_mapping"\s*:\s*"[^"]+"', '"config_sub_mapping": "config_sub_mapping_cpu.json"')
replace(config_json, r'"config_global_mapping"\s*:\s*"[^"]+"', '"config_global_mapping": "config_global_mapping_cpu.json"')

config_ros = config_dir / "config_ros.json"
modules = ['"libgnss_global.so"']
if viewer:
    modules = ['"libstandard_viewer.so"', '"librviz_viewer.so"', '"libgnss_global.so"']
replace(config_ros, r'"extension_modules"\s*:\s*\[[^\]]*\]', '"extension_modules": [\n      ' + ',\n      '.join(modules) + '\n    ]')
replace(config_ros, r'"imu_topic"\s*:\s*"[^"]+"', '"imu_topic": "/gps_p1/imu"')
replace(config_ros, r'"points_topic"\s*:\s*"[^"]+"', '"points_topic": "/luminar_front/points"')
replace(config_ros, r'"external_odom_topic"\s*:\s*"[^"]+"', '"external_odom_topic": "/gps_p1/filtered_odom_rtk_fixed"')
replace(config_ros, r'"points_qos"\s*:\s*\{[^}]*\}', '"points_qos": {\n      "profile": "sensor_data",\n      "depth": 5000,\n      "reliability": "reliable"\n    }')
replace(config_ros, r'"imu_qos"\s*:\s*\{[^}]*\}', '"imu_qos": {\n      "profile": "sensor_data",\n      "depth": 5000,\n      "reliability": "best_effort"\n    }')

config_sensors = config_dir / "config_sensors.json"
replace(config_sensors, r'"global_shutter_lidar"\s*:\s*(true|false)', '"global_shutter_lidar": false')
replace(config_sensors, r'"lidar_concat"\s*:\s*\{(.*?)"enabled"\s*:\s*(true|false)', lambda m: '"lidar_concat": {' + m.group(1) + '"enabled": true')
replace(config_sensors, r'"time_threshold"\s*:\s*[0-9.eE+-]+', '"time_threshold": 0.03')
replace(config_sensors, r'"buffer_size"\s*:\s*[0-9]+', '"buffer_size": 200')
PY

cat > "$output_dir/qos.yaml" <<'EOF'
/luminar_front/points:
  reliability: reliable
  history: keep_last
  depth: 5000
/luminar_left/points:
  reliability: reliable
  history: keep_last
  depth: 5000
/luminar_right/points:
  reliability: reliable
  history: keep_last
  depth: 5000
/atlas/pose_filtered:
  reliability: reliable
  history: keep_last
  depth: 5000
EOF

source /opt/ros/jazzy/setup.bash
source /home/roar/Documents/race_common/install/setup.bash
source "$repo_root/install/setup.bash"

export ROS_LOG_DIR="$output_dir/ros_log"
export ROS_HOME="$output_dir/ros_home"
export ROS_DOMAIN_ID="$domain_id"

log_state() {
  echo "$*" >> "$output_dir/state.txt"
}

log_runner() {
  echo "$* $(date -Iseconds)" >> "$output_dir/runner.log"
}

group_alive() {
  local pgid="$1"
  [ -n "$pgid" ] && kill -0 "-$pgid" 2>/dev/null
}

stop_group() {
  local pgid="$1"
  local sig="$2"
  if group_alive "$pgid"; then
    kill "-$sig" "-$pgid" 2>/dev/null || true
  fi
}

latest_imu_wait_signature() {
  if [ ! -f "$output_dir/glim.log" ]; then
    return 0
  fi

  tail -n 500 "$output_dir/glim.log" 2>/dev/null \
    | sed -nE 's/.*waiting for IMU data \(scan_end_time=([0-9.]+), last_imu_time=([0-9.]+) .*/\1 \2/p' \
    | tail -n 1 \
    | awk '{ if (NF == 2) printf "%.6f %.6f %.6f\n", $1, $2, $1 - $2 }' \
    || true
}

wait_for_tail_or_drain() {
  local start now elapsed sig last_sig="" sig_start=0 stable=0 reason="post_bag_drain_complete"
  start=$(date +%s)

  while group_alive "$GLIM_PGID"; do
    now=$(date +%s)
    elapsed=$((now - start))
    sig="$(latest_imu_wait_signature)"

    if [ -n "$sig" ]; then
      if [ "$sig" != "$last_sig" ]; then
        last_sig="$sig"
        sig_start="$now"
        stable=0
      else
        stable=$((now - sig_start))
      fi

      log_runner "tail_guard imu_wait scan_end_last_imu_delta=${sig} stable=${stable}s elapsed=${elapsed}s"
      if [ "$stable" -ge "$tail_wait_stall_sec" ]; then
        reason="imu_tail_stall scan_end_last_imu_delta=${sig} stable=${stable}s"
        break
      fi
    fi

    if [ "$elapsed" -ge "$post_bag_drain_sec" ]; then
      break
    fi

    sleep 5
  done

  log_state "tail_guard_stop reason=${reason} $(date -Iseconds)"
  log_runner "tail_guard_stop reason=${reason}"
}

wait_for_glim_exit() {
  local start waited glim_status=0

  start=$(date +%s)
  while group_alive "$GLIM_PGID"; do
    sleep 20
    waited=$(( $(date +%s) - start ))
    log_runner "waiting_glim_save=${waited}s"
    if [ "$waited" -ge "$glim_save_timeout_sec" ]; then
      log_state "glim_save_timeout ${waited}s $(date -Iseconds)"
      stop_group "$GLIM_PGID" TERM
      sleep 5
      stop_group "$GLIM_PGID" KILL
      break
    fi
  done

  wait "$GLIM_PGID" || glim_status=$?
  log_state "glim_done status=${glim_status} $(date -Iseconds)"
  return "$glim_status"
}

validate_dump() {
  local dump="$output_dir/dump"
  local missing=0 submap_count

  for required in graph.bin values.bin traj_lidar.txt T_world_utm.txt; do
    if [ ! -s "$dump/$required" ]; then
      echo "missing or empty dump artifact: $dump/$required" | tee -a "$output_dir/runner.log" >&2
      missing=1
    fi
  done

  submap_count=$(find "$dump" -maxdepth 1 -type d -regextype posix-extended -regex '.*/[0-9]{6}' 2>/dev/null | wc -l)
  if [ "$submap_count" -lt 1 ]; then
    echo "missing GLIM submap directories under: $dump" | tee -a "$output_dir/runner.log" >&2
    missing=1
  fi

  if [ "$missing" -eq 0 ]; then
    log_state "dump_validation status=ok submaps=${submap_count} $(date -Iseconds)"
    echo "dump validation: ok (${submap_count} submaps)"
    return 0
  fi

  log_state "dump_validation status=failed submaps=${submap_count} $(date -Iseconds)"
  return 1
}

cleanup() {
  set +e
  log_state "cleanup_signal $(date -Iseconds)"
  stop_group "${BAG_PGID:-}" INT
  stop_group "${GLIM_PGID:-}" INT
  stop_group "${ADAPTER_PGID:-}" INT
}
trap cleanup INT TERM

echo "RUN=$output_dir" > "$output_dir/state.txt"
log_state "BAG_DIR=$bag_dir"
log_state "PCAP_PATH=$pcap_path"
log_state "CONFIG_PATH=$config_dir"
log_state "RATE=$rate"
log_state "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

setsid bash -lc "exec ros2 launch adapter adapter.launch.py p1_imu_pcap_path:='$pcap_path' use_sim_time:=true summary_output_path:='$output_dir/adapter_summary.txt'" > "$output_dir/adapter.log" 2>&1 &
ADAPTER_PGID=$!
log_state "ADAPTER_PGID=$ADAPTER_PGID"
sleep 3

setsid bash -lc "exec '$repo_root/install/glim_ros/lib/glim_ros/glim_rosnode' --ros-args -p config_path:='$config_dir' -p dump_path:='$output_dir/dump' -p deterministic_live_input:=true -p live_input_max_queue_size:=50000" > "$output_dir/glim.log" 2>&1 &
GLIM_PGID=$!
log_state "GLIM_PGID=$GLIM_PGID"
sleep 5

BAG_CMD=(ros2 bag play "$bag_dir" --clock 100 --rate "$rate" --disable-keyboard-controls --read-ahead-queue-size 20000 --qos-profile-overrides-path "$output_dir/qos.yaml" --topics /atlas/pose_filtered /luminar_front/points /luminar_left/points /luminar_right/points)
if [ -n "$duration" ]; then
  BAG_CMD+=(--playback-duration "$duration")
fi

setsid bash -lc "exec \"\$@\"" _ "${BAG_CMD[@]}" > "$output_dir/bag_play.log" 2>&1 &
BAG_PGID=$!
log_state "BAG_PGID=$BAG_PGID"

wait "$BAG_PGID" || true
log_state "bag_done $(date -Iseconds)"

wait_for_tail_or_drain

stop_group "$BAG_PGID" INT
stop_group "$ADAPTER_PGID" INT
stop_group "$GLIM_PGID" INT

wait_for_glim_exit || true

sleep 2
stop_group "$ADAPTER_PGID" TERM
wait "$ADAPTER_PGID" || true
log_state "adapter_done $(date -Iseconds)"

validate_dump

echo "dump: $output_dir/dump"
