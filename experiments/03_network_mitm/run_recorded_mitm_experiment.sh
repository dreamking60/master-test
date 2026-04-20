#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
BASE_OUT_DIR="$PROJECT_ROOT/logs/experiments/03_network_mitm"
OUT_DIR="$BASE_OUT_DIR/recorded_$STAMP"

BASELINE_SECONDS="${BASELINE_SECONDS:-15}"
ATTACK_SECONDS="${ATTACK_SECONDS:-45}"
AFTER_SECONDS="${AFTER_SECONDS:-15}"
ATTACK_DELAY_MS="${ATTACK_DELAY_MS:-500}"
ATTACK_LOSS_PERCENT="${ATTACK_LOSS_PERCENT:-0}"

mkdir -p "$PROJECT_ROOT/logs/experiments"
if [ -e "$BASE_OUT_DIR" ] && [ ! -w "$BASE_OUT_DIR" ]; then
  echo "Fixing log directory ownership: $BASE_OUT_DIR"
  sudo chown -R "$(id -u):$(id -g)" "$BASE_OUT_DIR"
fi
mkdir -p "$OUT_DIR"
cd "$PROJECT_ROOT"

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$OUT_DIR/run.log"
}

run_capture() {
  local title="$1"
  local file="$2"
  shift 2
  {
    echo "# $title"
    echo "# ts=$(date --iso-8601=seconds)"
    echo "# cmd=$*"
    "$@"
  } > "$OUT_DIR/$file" 2>&1 || true
}

mitm_exec() {
  sudo ./scripts/wsl_docker/mitm_exec.sh "$@"
}

cleanup_background() {
  set +e
  for pid_file in "$OUT_DIR"/*.pid; do
    [ -f "$pid_file" ] || continue
    pid="$(cat "$pid_file" 2>/dev/null)"
    [ -n "$pid" ] && kill "$pid" 2>/dev/null
  done
  wait 2>/dev/null
  sudo ./scripts/wsl_docker/mitm_exec.sh attacker "tc qdisc del dev eth0 root 2>/dev/null || true" >/dev/null 2>&1 || true
  sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration 1 --log-dir /tmp >/dev/null 2>&1 || true
}
trap cleanup_background EXIT

{
  echo "# Recorded Network MITM Experiment"
  echo "timestamp=$STAMP"
  echo "baseline_seconds=$BASELINE_SECONDS"
  echo "attack_seconds=$ATTACK_SECONDS"
  echo "after_seconds=$AFTER_SECONDS"
  echo "attack_delay_ms=$ATTACK_DELAY_MS"
  echo "attack_loss_percent=$ATTACK_LOSS_PERCENT"
} > "$OUT_DIR/metadata.txt"

log "Starting MITM Docker lab"
sudo ./scripts/wsl_docker/mitm_up.sh | tee -a "$OUT_DIR/run.log"

log "Capturing initial topology"
run_capture "docker ps" "00_docker_ps.txt" sudo docker ps --format 'table {{.Names}}\t{{.Networks}}\t{{.Status}}'
run_capture "docker network inspect" "00_network_inspect.json" sudo docker network inspect wsl_docker_mitm_lab
run_capture "controller initial state" "00_controller_state.txt" sudo ./scripts/wsl_docker/mitm_exec.sh controller "hostname; ip -br addr; ip route; ip neigh"
run_capture "robot initial state" "00_robot_state.txt" sudo ./scripts/wsl_docker/mitm_exec.sh robot "hostname; ip -br addr; ip route; ip neigh"
run_capture "attacker initial state" "00_attacker_state.txt" sudo ./scripts/wsl_docker/mitm_exec.sh attacker "hostname; ip -br addr; ip route; ip neigh; cat /proc/sys/net/ipv4/ip_forward; tc qdisc show dev eth0"

log "Starting ROS2 robot subscriber, controller publisher, ping, and attacker tcpdump"
mitm_exec robot "source /opt/ros/jazzy/setup.bash && python3 /workspace/project/scripts/wsl_docker/mitm_robot_cmd_sub.py" \
  > "$OUT_DIR/ros2_robot_subscriber.log" 2>&1 & echo $! > "$OUT_DIR/ros2_robot_subscriber.pid"
sleep 2
mitm_exec controller "source /opt/ros/jazzy/setup.bash && python3 /workspace/project/scripts/wsl_docker/mitm_controller_cmd_pub.py" \
  > "$OUT_DIR/ros2_controller_publisher.log" 2>&1 & echo $! > "$OUT_DIR/ros2_controller_publisher.pid"
mitm_exec controller "ping -D 172.28.0.20" \
  > "$OUT_DIR/ping_controller_to_robot.log" 2>&1 & echo $! > "$OUT_DIR/ping_controller_to_robot.pid"
mitm_exec attacker "tcpdump -n -tt -i eth0 'host 172.28.0.10 and host 172.28.0.20'" \
  > "$OUT_DIR/tcpdump_attacker_controller_robot.log" 2>&1 & echo $! > "$OUT_DIR/tcpdump_attacker_controller_robot.pid"
mitm_exec robot "tcpdump -n -tt -i eth0 'arp or icmp or udp portrange 7400-7600'" \
  > "$OUT_DIR/tcpdump_robot_arp_icmp_dds.log" 2>&1 & echo $! > "$OUT_DIR/tcpdump_robot_arp_icmp_dds.pid"

log "Baseline phase: ${BASELINE_SECONDS}s"
sleep "$BASELINE_SECONDS"
run_capture "controller ARP before attack" "01_before_controller_arp.txt" sudo ./scripts/wsl_docker/mitm_exec.sh controller "ip neigh"
run_capture "robot ARP before attack" "01_before_robot_arp.txt" sudo ./scripts/wsl_docker/mitm_exec.sh robot "ip neigh"
run_capture "attacker qdisc before attack" "01_before_attacker_qdisc.txt" sudo ./scripts/wsl_docker/mitm_exec.sh attacker "cat /proc/sys/net/ipv4/ip_forward; tc qdisc show dev eth0"

log "Attack phase: delay=${ATTACK_DELAY_MS}ms loss=${ATTACK_LOSS_PERCENT}% duration=${ATTACK_SECONDS}s"
mitm_exec attacker "bash /workspace/project/scripts/wsl_docker/mitm_enable_forward_delay.sh '$ATTACK_DELAY_MS' '$ATTACK_LOSS_PERCENT'" \
  > "$OUT_DIR/02_attack_enable_forward_delay.log" 2>&1
sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration "$ATTACK_SECONDS" \
  --log-dir /tmp \
  > "$OUT_DIR/02_attack_arp_poison.log" 2>&1 & echo $! > "$OUT_DIR/arp_poison.pid"
sleep 5
run_capture "controller ARP during attack" "02_during_controller_arp.txt" sudo ./scripts/wsl_docker/mitm_exec.sh controller "ip neigh"
run_capture "robot ARP during attack" "02_during_robot_arp.txt" sudo ./scripts/wsl_docker/mitm_exec.sh robot "ip neigh"
run_capture "attacker qdisc during attack" "02_during_attacker_qdisc.txt" sudo ./scripts/wsl_docker/mitm_exec.sh attacker "cat /proc/sys/net/ipv4/ip_forward; tc qdisc show dev eth0"
wait "$(cat "$OUT_DIR/arp_poison.pid")" || true

log "After phase: ${AFTER_SECONDS}s"
sudo ./scripts/wsl_docker/mitm_exec.sh attacker "tc qdisc del dev eth0 root 2>/dev/null || true" >/dev/null 2>&1 || true
sleep "$AFTER_SECONDS"
run_capture "controller ARP after restore" "03_after_controller_arp.txt" sudo ./scripts/wsl_docker/mitm_exec.sh controller "ip neigh"
run_capture "robot ARP after restore" "03_after_robot_arp.txt" sudo ./scripts/wsl_docker/mitm_exec.sh robot "ip neigh"
run_capture "attacker qdisc after restore" "03_after_attacker_qdisc.txt" sudo ./scripts/wsl_docker/mitm_exec.sh attacker "cat /proc/sys/net/ipv4/ip_forward; tc qdisc show dev eth0"

log "Stopping background capture processes"
cleanup_background
trap - EXIT

log "Generating summary"
python3 - "$OUT_DIR" <<'PY' > "$OUT_DIR/summary.md"
import pathlib
import re
import statistics
import sys

out = pathlib.Path(sys.argv[1])

ping_re = re.compile(r'time=([0-9.]+) ms')
lat_re = re.compile(r'last_latency_ms=([0-9.]+).*avg_latency_ms=([0-9.]+).*max_latency_ms=([0-9.]+).*max_gap_ms=([0-9.]+)')

def floats_from(path, regex, group=1):
    values = []
    if not path.exists():
        return values
    for line in path.read_text(errors='ignore').splitlines():
        m = regex.search(line)
        if m:
            values.append(float(m.group(group)))
    return values

def describe(values):
    if not values:
        return 'n/a'
    return f'count={len(values)} min={min(values):.1f} avg={statistics.mean(values):.1f} max={max(values):.1f}'

ping = floats_from(out / 'ping_controller_to_robot.log', ping_re)
lat_lines = []
lat_last = []
lat_avg = []
lat_max = []
gaps = []
sub = out / 'ros2_robot_subscriber.log'
if sub.exists():
    for line in sub.read_text(errors='ignore').splitlines():
        m = lat_re.search(line)
        if m:
            lat_last.append(float(m.group(1)))
            lat_avg.append(float(m.group(2)))
            lat_max.append(float(m.group(3)))
            gaps.append(float(m.group(4)))
            lat_lines.append(line)

print('# Recorded MITM Experiment Summary')
print()
print(f'Output directory: `{out}`')
print()
print('## Ping RTT')
print(describe(ping))
print()
print('## ROS2 /mitm_cmd Subscriber Latency')
print(f'last_latency_ms: {describe(lat_last)}')
print(f'avg_latency_ms samples: {describe(lat_avg)}')
print(f'max_latency_ms samples: {describe(lat_max)}')
print(f'max_gap_ms samples: {describe(gaps)}')
print()
print('## Key Evidence Files')
for name in [
    '01_before_controller_arp.txt',
    '02_during_controller_arp.txt',
    '03_after_controller_arp.txt',
    '01_before_robot_arp.txt',
    '02_during_robot_arp.txt',
    '03_after_robot_arp.txt',
    '02_attack_enable_forward_delay.log',
    '02_attack_arp_poison.log',
    'tcpdump_attacker_controller_robot.log',
    'ros2_robot_subscriber.log',
]:
    print(f'- `{name}`')
PY

cat "$OUT_DIR/summary.md"
log "Evidence directory: $OUT_DIR"
