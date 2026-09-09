#!/usr/bin/env bash
set -euo pipefail

WITH_DRIVER=1
OUTPUT_ROOT="${HOME}/agt_r3_runs"
SESSION_LABEL="r3_drive"
STATIC_WINDOW_SEC="60"
SNAPSHOT_PERIOD_SEC="5"

usage() {
  cat <<'EOF'
Usage: r3_drive_validation.sh [options]

Options:
  --no-driver                 Do not launch asensing_driver (use when runtime already owns the serial device)
  --output-root DIR           Root directory for test runs (default: ~/agt_r3_runs)
  --label NAME                Session label (default: r3_drive)
  --static-window SEC         Initial stationary window after first usable GNSS fix (default: 60)
  --snapshot-period SEC       report.json/report.md refresh period (default: 5)
  -h, --help                  Show this help

The script NEVER sends vehicle motion commands. It starts only sensor/monitor/rosbag processes.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-driver) WITH_DRIVER=0; shift ;;
    --output-root) OUTPUT_ROOT="$2"; shift 2 ;;
    --label) SESSION_LABEL="$2"; shift 2 ;;
    --static-window) STATIC_WINDOW_SEC="$2"; shift 2 ;;
    --snapshot-period) SNAPSHOT_PERIOD_SEC="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not in PATH. Source ROS 2 Humble and this workspace first." >&2
  exit 1
fi

SAFE_LABEL="$(printf '%s' "$SESSION_LABEL" | tr -cs 'A-Za-z0-9._-' '_')"
RUN_ID="$(date +%Y%m%d_%H%M%S)_${SAFE_LABEL}"
RUN_DIR="${OUTPUT_ROOT}/${RUN_ID}"
mkdir -p "$RUN_DIR"

cat >"${RUN_DIR}/session.txt" <<EOF
run_id=${RUN_ID}
session_label=${SESSION_LABEL}
start_time=$(date --iso-8601=seconds)
with_driver=${WITH_DRIVER}
static_window_sec=${STATIC_WINDOW_SEC}
snapshot_period_sec=${SNAPSHOT_PERIOD_SEC}
ros_distro=${ROS_DISTRO:-unknown}
EOF

DRIVER_PID=""
MONITOR_PID=""
BAG_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  echo
  echo "Stopping R3 validation processes..."

  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$MONITOR_PID" ]] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    kill -INT "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
  if [[ -n "$DRIVER_PID" ]] && kill -0 "$DRIVER_PID" 2>/dev/null; then
    kill -INT "$DRIVER_PID" 2>/dev/null || true
    wait "$DRIVER_PID" 2>/dev/null || true
  fi

  {
    echo "stop_time=$(date --iso-8601=seconds)"
    echo "exit_code=${exit_code}"
  } >>"${RUN_DIR}/session.txt"

  echo "R3 evidence directory: ${RUN_DIR}"
  echo "  report: ${RUN_DIR}/report.md"
  echo "  json:   ${RUN_DIR}/report.json"
  echo "  bag:    ${RUN_DIR}/bag"
  exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [[ "$WITH_DRIVER" -eq 1 ]]; then
  echo "Starting ASENSING driver..."
  ros2 launch agt_asensing_driver asensing.launch.py >"${RUN_DIR}/driver.log" 2>&1 &
  DRIVER_PID=$!
  sleep 2
fi

echo "Starting R3 monitor..."
ros2 run agt_asensing_driver r3_hardware_monitor --ros-args \
  -p output_dir:="${RUN_DIR}" \
  -p session_label:="${SESSION_LABEL}" \
  -p initial_static_window_sec:="${STATIC_WINDOW_SEC}" \
  -p snapshot_period_sec:="${SNAPSHOT_PERIOD_SEC}" \
  >"${RUN_DIR}/monitor.log" 2>&1 &
MONITOR_PID=$!
sleep 1

echo "Starting rosbag2..."
ros2 bag record \
  -o "${RUN_DIR}/bag" \
  /ins/navsatfix \
  /ins/status \
  /ins/imu \
  /ins/velocity \
  /ins/r3/marker \
  /tf_static \
  >"${RUN_DIR}/bag.log" 2>&1 &
BAG_PID=$!

cat <<EOF

R3 validation is running.

Safety boundary:
  - this workflow NEVER publishes cmd_vel;
  - do not unplug antennas/serial cables while the vehicle is moving;
  - perform reconnect/degradation manipulations only while stationary and safe.

Operator flow:
  1. Keep the vehicle stationary until the initial ${STATIC_WINDOW_SEC}s window has completed.
  2. Drive the controlled test route normally.
  3. If safe, include an RTK quality degradation/recovery segment.
  4. Stop the vehicle before any serial reconnect test.
  5. Press Ctrl-C here when the run is complete.

Live report:
  ${RUN_DIR}/report.md

Optional event marker (Codex/operator may issue from another terminal):
  ros2 topic pub --once /ins/r3/marker std_msgs/msg/String "{data: 'CONTROLLED_EVENT'}"

EOF

while true; do
  if ! kill -0 "$MONITOR_PID" 2>/dev/null; then
    echo "R3 monitor exited unexpectedly. See ${RUN_DIR}/monitor.log" >&2
    exit 1
  fi
  if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo "rosbag2 exited unexpectedly. See ${RUN_DIR}/bag.log" >&2
    exit 1
  fi
  if [[ "$WITH_DRIVER" -eq 1 ]] && ! kill -0 "$DRIVER_PID" 2>/dev/null; then
    echo "ASENSING driver exited unexpectedly. See ${RUN_DIR}/driver.log" >&2
    exit 1
  fi
  sleep 1
done
