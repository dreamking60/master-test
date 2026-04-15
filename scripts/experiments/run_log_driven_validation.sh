#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

MODE="${1:-all}"
BASELINE_SECONDS="${BASELINE_SECONDS:-15}"
ATTACK_SECONDS="${ATTACK_SECONDS:-25}"
AFTER_SECONDS="${AFTER_SECONDS:-8}"
STARTUP_SECONDS="${STARTUP_SECONDS:-18}"
DOS_PAYLOAD_SIZE="${DOS_PAYLOAD_SIZE:-1200}"
DOS_THREADS_PER_PORT="${DOS_THREADS_PER_PORT:-3}"
DOS_RATE="${DOS_RATE:-0}"
DOS_LABEL="${DOS_LABEL:-level1_baseline}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ROOT_OUT="$PROJECT_ROOT/logs/experiments/log_driven_validation/$STAMP"

usage() {
  cat <<EOF
Usage:
  $0 [open-injection|sros2-dos|all]

Environment:
  BASELINE_SECONDS=15
  ATTACK_SECONDS=25
  AFTER_SECONDS=8
  STARTUP_SECONDS=18
  DOS_PAYLOAD_SIZE=1200
  DOS_THREADS_PER_PORT=3
  DOS_RATE=0
  DOS_LABEL=level1_baseline

Outputs:
  logs/experiments/log_driven_validation/<timestamp>/<scenario>/
EOF
}

if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "$MODE" != "open-injection" && "$MODE" != "sros2-dos" && "$MODE" != "all" ]]; then
  echo "ERROR: mode must be open-injection, sros2-dos, or all." >&2
  usage >&2
  exit 1
fi

set +u
source "$PROJECT_ROOT/scripts/setup/source_wsl_ros_env.sh" >/dev/null
set -u

mkdir -p "$ROOT_OUT"

GAZEBO_PID=""
CONTROLLER_PID=""
ATTACKER_PID=""
RECORDER_PID=""
MONITOR_PID=""

log() {
  printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

sudo_cmd() {
  sudo -n "$@"
}

preflight_sudo() {
  local out_dir="$1"
  local failed=0
  {
    echo "Preflight: checking non-interactive sudo access for experiment automation"
    echo "timestamp=$(date '+%Y-%m-%d %H:%M:%S')"
    echo
    for script in \
      "$PROJECT_ROOT/scripts/wsl_docker/down.sh" \
      "$PROJECT_ROOT/scripts/wsl_docker/up.sh" \
      "$PROJECT_ROOT/scripts/wsl_docker/secure_up.sh" \
      "$PROJECT_ROOT/scripts/wsl_docker/run_controller.sh" \
      "$PROJECT_ROOT/scripts/wsl_docker/exec_attacker.sh" \
      "$PROJECT_ROOT/scripts/wsl_docker/docker_status.sh"; do
      echo "--- sudo -n -l $script ---"
      if sudo -n -l "$script" >/dev/null 2>&1; then
        echo "OK: $script"
      else
        echo "MISSING: $script"
        failed=1
      fi
    done
    echo
    if [[ "$failed" == "1" ]]; then
      cat <<EOF
Non-interactive sudo is not configured for all required scripts.

Run once in a real WSL terminal:
  cd "$PROJECT_ROOT"
  sudo ./scripts/setup/install_codex_sudoers.sh

Then verify:
  sudo -n "$PROJECT_ROOT/scripts/wsl_docker/down.sh"
  sudo -n "$PROJECT_ROOT/scripts/wsl_docker/docker_status.sh" tb3-controller tb3-attacker
EOF
    else
      echo "Preflight passed."
    fi
  } > "$out_dir/preflight.log" 2>&1

  if [[ "$failed" == "1" ]]; then
    cat "$out_dir/preflight.log" >&2
    return 1
  fi
}

stop_pid() {
  local pid="$1"
  local name="$2"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    log "Stopping $name pid=$pid"
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup_runtime() {
  stop_pid "$ATTACKER_PID" "attacker"
  stop_pid "$CONTROLLER_PID" "controller"
  stop_pid "$RECORDER_PID" "trajectory recorder"
  stop_pid "$MONITOR_PID" "monitor"
  stop_pid "$GAZEBO_PID" "gazebo"
  sudo_cmd "$PROJECT_ROOT/scripts/wsl_docker/down.sh" >/dev/null 2>&1 || true
  pkill -f "turtlebot3_empty_world_custom_bridge.launch.py" 2>/dev/null || true
  pkill -f "cmd_vel_relay.py" 2>/dev/null || true
  pkill -f "trajectory_trail_spawner.py" 2>/dev/null || true
}

trap cleanup_runtime EXIT

run_project_cleanup() {
  log "Cleaning previous experiment state"
  "$PROJECT_ROOT/scripts/demo/cleanup_all_experiments.sh" > "$1/cleanup.log" 2>&1 || true
}

start_monitor() {
  local out_dir="$1"
  (
    while true; do
      {
        printf '\n=== %s ===\n' "$(date '+%Y-%m-%d %H:%M:%S')"
        printf '\n--- docker status ---\n'
        docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' 2>&1 && \
          docker stats --no-stream tb3-controller tb3-attacker 2>&1 || \
          sudo -n "$PROJECT_ROOT/scripts/wsl_docker/docker_status.sh" tb3-controller tb3-attacker 2>&1 || true
        printf '\n--- DDS UDP sockets ---\n'
        ss -lunp 2>&1 | grep -E ':(74[0-9][0-9]|75[0-9][0-9])' || true
        printf '\n--- ROS topic info /cmd_vel_in ---\n'
        ros2 topic info /cmd_vel_in -v 2>&1 || true
        printf '\n--- ROS topic hz /odom short sample ---\n'
        timeout 3 ros2 topic hz /odom 2>&1 || true
      } >> "$out_dir/resource_monitor.log"
      sleep 2
    done
  ) &
  MONITOR_PID=$!
}

start_gazebo() {
  local secure="$1"
  local out_dir="$2"
  rm -f "$PROJECT_ROOT/test.log"
  if [[ "$secure" == "1" ]]; then
    "$PROJECT_ROOT/scripts/setup/run_wsl_gazebo_secure.sh" > "$out_dir/gazebo_wrapper.log" 2>&1 &
  else
    "$PROJECT_ROOT/scripts/setup/run_wsl_gazebo.sh" > "$out_dir/gazebo_wrapper.log" 2>&1 &
  fi
  GAZEBO_PID=$!
  log "Gazebo started pid=$GAZEBO_PID secure=$secure; waiting ${STARTUP_SECONDS}s"
  sleep "$STARTUP_SECONDS"
}

start_recorder() {
  local out_dir="$1"
  local duration
  duration="$(python3 - <<PY
print(float("$BASELINE_SECONDS") + float("$ATTACK_SECONDS") + float("$AFTER_SECONDS"))
PY
)"
  python3 "$PROJECT_ROOT/attack_experiments/scripts/record_trajectory.py" \
    --output "$out_dir/trajectory.csv" \
    --duration "$duration" \
    --plot > "$out_dir/trajectory_recorder.log" 2>&1 &
  RECORDER_PID=$!
  log "Trajectory recorder started pid=$RECORDER_PID duration=${duration}s"
}

collect_snapshot() {
  local out_dir="$1"
  local label="$2"
  {
    echo "snapshot=$label"
    echo "timestamp=$(date '+%Y-%m-%d %H:%M:%S')"
    echo
    echo "--- topics ---"
    ros2 topic list || true
    echo
    echo "--- /cmd_vel_in ---"
    ros2 topic info /cmd_vel_in -v || true
    echo
    echo "--- /cmd_vel ---"
    ros2 topic info /cmd_vel -v || true
    echo
    echo "--- /odom hz sample ---"
    timeout 4 ros2 topic hz /odom || true
  } > "$out_dir/snapshot_${label}.txt" 2>&1
}

copy_logs() {
  local out_dir="$1"
  cp "$PROJECT_ROOT/test.log" "$out_dir/test.log" 2>/dev/null || true
  cp "$PROJECT_ROOT/logs/wsl_docker/controller_pub.log" "$out_dir/controller_pub.log" 2>/dev/null || true
  cp "$PROJECT_ROOT/logs/wsl_docker/attacker_pub.log" "$out_dir/attacker_pub.log" 2>/dev/null || true
  tail -n 240 "$PROJECT_ROOT/test.log" > "$out_dir/test_tail.log" 2>/dev/null || true
  grep -E "trajectory|Spawned|marker|Failed|cmd_vel_relay|relayed=|ERROR|WARN" "$PROJECT_ROOT/test.log" > "$out_dir/test_markers.log" 2>/dev/null || true
}

summarize_run() {
  local out_dir="$1"
  local label="$2"
  if [[ -s "$out_dir/trajectory.csv" ]]; then
    python3 "$PROJECT_ROOT/scripts/experiments/summarize_trajectory.py" \
      "$out_dir/trajectory.csv" \
      --label "$label" \
      --baseline-seconds "$BASELINE_SECONDS" \
      --attack-seconds "$ATTACK_SECONDS" \
      --after-seconds "$AFTER_SECONDS" \
      --output "$out_dir/trajectory_summary.txt" > "$out_dir/trajectory_summary.stdout" 2>&1 || true
  else
    echo "No trajectory.csv generated." > "$out_dir/trajectory_summary.txt"
  fi
}

run_open_injection() {
  local out_dir="$ROOT_OUT/open_injection"
  mkdir -p "$out_dir"
  log "=== Running open injection experiment ==="
  preflight_sudo "$out_dir"
  run_project_cleanup "$out_dir"
  start_gazebo 0 "$out_dir"
  start_monitor "$out_dir"
  sudo_cmd "$PROJECT_ROOT/scripts/wsl_docker/up.sh" > "$out_dir/docker_up.log" 2>&1
  start_recorder "$out_dir"
  collect_snapshot "$out_dir" "baseline_start"
  sudo_cmd -E "$PROJECT_ROOT/scripts/wsl_docker/run_controller.sh" > "$out_dir/controller_session.log" 2>&1 &
  CONTROLLER_PID=$!
  log "Controller started pid=$CONTROLLER_PID"
  sleep "$BASELINE_SECONDS"
  collect_snapshot "$out_dir" "before_attack"
  sudo_cmd "$PROJECT_ROOT/scripts/wsl_docker/run_attacker.sh" > "$out_dir/attacker_session.log" 2>&1 &
  ATTACKER_PID=$!
  log "Attacker started pid=$ATTACKER_PID"
  sleep "$ATTACK_SECONDS"
  collect_snapshot "$out_dir" "during_attack"
  stop_pid "$ATTACKER_PID" "open attacker"
  ATTACKER_PID=""
  sleep "$AFTER_SECONDS"
  collect_snapshot "$out_dir" "after_attack"
  stop_pid "$CONTROLLER_PID" "open controller"
  CONTROLLER_PID=""
  wait "$RECORDER_PID" 2>/dev/null || true
  RECORDER_PID=""
  copy_logs "$out_dir"
  summarize_run "$out_dir" "open-injection"
  cleanup_runtime
  log "Open injection evidence: $out_dir"
}

run_sros2_dos() {
  local out_dir="$ROOT_OUT/sros2_dos"
  mkdir -p "$out_dir"
  log "=== Running SROS2 DoS experiment ==="
  {
    echo "DOS_LABEL=$DOS_LABEL"
    echo "BASELINE_SECONDS=$BASELINE_SECONDS"
    echo "ATTACK_SECONDS=$ATTACK_SECONDS"
    echo "AFTER_SECONDS=$AFTER_SECONDS"
    echo "STARTUP_SECONDS=$STARTUP_SECONDS"
    echo "DOS_PAYLOAD_SIZE=$DOS_PAYLOAD_SIZE"
    echo "DOS_THREADS_PER_PORT=$DOS_THREADS_PER_PORT"
    echo "DOS_RATE=$DOS_RATE"
  } > "$out_dir/experiment_config.env"
  preflight_sudo "$out_dir"
  run_project_cleanup "$out_dir"
  if [[ "${FORCE_SROS2_INIT:-0}" == "1" ]] || \
     [[ ! -f "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/controller/permissions.p7s" ]] || \
     [[ ! -f "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/gazebo/permissions.p7s" ]]; then
    sudo_cmd "$PROJECT_ROOT/scripts/wsl_docker/init_sros2_docker.sh" > "$out_dir/init_sros2.log" 2>&1
  else
    echo "Existing SROS2 artifacts found; set FORCE_SROS2_INIT=1 to regenerate." > "$out_dir/init_sros2.log"
  fi
  start_gazebo 1 "$out_dir"
  start_monitor "$out_dir"
  sudo_cmd -E "$PROJECT_ROOT/scripts/wsl_docker/secure_up.sh" > "$out_dir/secure_up.log" 2>&1
  start_recorder "$out_dir"
  collect_snapshot "$out_dir" "baseline_start"
  CONTROLLER_PATTERN=square \
  CONTROLLER_LINEAR_SPEED=0.2 \
  CONTROLLER_ANGULAR_SPEED=0.8 \
  CONTROLLER_SQUARE_SIDE_SECONDS=4.0 \
  CONTROLLER_SQUARE_TURN_SECONDS=1.96 \
    sudo_cmd -E "$PROJECT_ROOT/scripts/wsl_docker/run_controller.sh" > "$out_dir/controller_session.log" 2>&1 &
  CONTROLLER_PID=$!
  log "Secure square controller started pid=$CONTROLLER_PID"
  sleep "$BASELINE_SECONDS"
  collect_snapshot "$out_dir" "before_attack"
  sudo_cmd "$PROJECT_ROOT/scripts/wsl_docker/exec_attacker.sh" \
    python3 /workspace/project/attack_experiments/scripts/dos_attack_sros2.py \
    --discover-ports \
    --duration "$ATTACK_SECONDS" \
    --payload-size "$DOS_PAYLOAD_SIZE" \
    --threads-per-port "$DOS_THREADS_PER_PORT" \
    --rate "$DOS_RATE" \
    > "$out_dir/dos_attack.log" 2>&1 &
  ATTACKER_PID=$!
  log "DoS attacker started pid=$ATTACKER_PID"
  wait "$ATTACKER_PID" || true
  ATTACKER_PID=""
  collect_snapshot "$out_dir" "during_attack"
  sleep "$AFTER_SECONDS"
  collect_snapshot "$out_dir" "after_attack"
  stop_pid "$CONTROLLER_PID" "secure controller"
  CONTROLLER_PID=""
  wait "$RECORDER_PID" 2>/dev/null || true
  RECORDER_PID=""
  copy_logs "$out_dir"
  summarize_run "$out_dir" "sros2-dos"
  cleanup_runtime
  log "SROS2 DoS evidence: $out_dir"
}

case "$MODE" in
  open-injection)
    run_open_injection
    ;;
  sros2-dos)
    run_sros2_dos
    ;;
  all)
    run_open_injection
    run_sros2_dos
    ;;
esac

log "Validation complete. Evidence root: $ROOT_OUT"
