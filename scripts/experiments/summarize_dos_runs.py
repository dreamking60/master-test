#!/usr/bin/env python3
import argparse
import re
from pathlib import Path


PACKET_RE = re.compile(r"Finished\. Sent (\d+) packets in ([0-9.]+)s, avg (\d+) pkt/s")
CPU_RE = re.compile(r"tb3-attacker\s+([0-9.]+)%")
PUBLISHED_RE = re.compile(r"\[([0-9.]+)\].*published=(\d+)")
WATCHDOG_RE = re.compile(r"No /cmd_vel_in for ([0-9.]+)s; published zero /cmd_vel")


def read_text(path):
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return ""


def parse_config(run_dir):
    config = {}
    for line in read_text(run_dir / "experiment_config.env").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            config[key] = value
    return config


def parse_attack(run_dir):
    text = read_text(run_dir / "dos_attack.log")
    match = PACKET_RE.search(text)
    if not match:
        return {"packets": "", "duration": "", "pps": ""}
    return {
        "packets": int(match.group(1)),
        "duration": float(match.group(2)),
        "pps": int(match.group(3)),
    }


def parse_max_attacker_cpu(run_dir):
    text = read_text(run_dir / "resource_monitor.log")
    values = [float(match.group(1)) for match in CPU_RE.finditer(text)]
    return max(values) if values else ""


def parse_publish_stalls(run_dir):
    text = read_text(run_dir / "controller_session.log")
    points = [(float(match.group(1)), int(match.group(2))) for match in PUBLISHED_RE.finditer(text)]
    stalls = []
    for (prev_t, prev_count), (cur_t, cur_count) in zip(points, points[1:]):
        elapsed = cur_t - prev_t
        count_delta = cur_count - prev_count
        expected = elapsed * 10.0
        if elapsed >= 1.5 and count_delta < expected * 0.5:
            stalls.append((elapsed, count_delta, expected))
    if not stalls:
        return {"stall_count": 0, "max_stall_s": 0.0}
    return {
        "stall_count": len(stalls),
        "max_stall_s": max(stall[0] for stall in stalls),
    }


def parse_watchdog(run_dir):
    text = read_text(run_dir / "test.log")
    if not text:
        text = read_text(run_dir / "test_markers.log")
    durations = [float(match.group(1)) for match in WATCHDOG_RE.finditer(text)]
    if not durations:
        return {"watchdog_count": 0, "max_watchdog_s": 0.0}
    return {
        "watchdog_count": len(durations),
        "max_watchdog_s": max(durations),
    }


def summarize_run(run_dir):
    config = parse_config(run_dir)
    attack = parse_attack(run_dir)
    stalls = parse_publish_stalls(run_dir)
    watchdog = parse_watchdog(run_dir)
    return {
        "run_dir": str(run_dir),
        "label": config.get("DOS_LABEL", ""),
        "threads": config.get("DOS_THREADS_PER_PORT", ""),
        "payload": config.get("DOS_PAYLOAD_SIZE", ""),
        "attack_seconds": config.get("ATTACK_SECONDS", ""),
        "packets": attack["packets"],
        "pps": attack["pps"],
        "max_attacker_cpu": parse_max_attacker_cpu(run_dir),
        "stall_count": stalls["stall_count"],
        "max_stall_s": stalls["max_stall_s"],
        "watchdog_count": watchdog["watchdog_count"],
        "max_watchdog_s": watchdog["max_watchdog_s"],
    }


def markdown_table(rows):
    headers = [
        "label",
        "threads",
        "payload",
        "attack_s",
        "packets",
        "pps",
        "max_attacker_cpu",
        "stalls",
        "max_stall_s",
        "watchdogs",
        "max_watchdog_s",
    ]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        values = [
            row["label"],
            row["threads"],
            row["payload"],
            row["attack_seconds"],
            row["packets"],
            row["pps"],
            f"{row['max_attacker_cpu']:.2f}%" if isinstance(row["max_attacker_cpu"], float) else "",
            row["stall_count"],
            f"{row['max_stall_s']:.2f}",
            row["watchdog_count"],
            f"{row['max_watchdog_s']:.2f}",
        ]
        lines.append("| " + " | ".join(str(value) for value in values) + " |")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Summarize SROS2 DoS run directories.")
    parser.add_argument("run_dirs", nargs="*", help="sros2_dos evidence directories")
    parser.add_argument("--latest", type=int, default=0, help="Use latest N sros2_dos dirs if no explicit dirs are provided.")
    args = parser.parse_args()

    if args.run_dirs:
        run_dirs = [Path(path) for path in args.run_dirs]
    else:
        count = args.latest or 3
        base = Path("logs/experiments/log_driven_validation")
        run_dirs = sorted(base.glob("*/sros2_dos"), key=lambda path: path.stat().st_mtime)[-count:]

    rows = [summarize_run(path) for path in run_dirs]
    print(markdown_table(rows))
    print()
    print("Run directories:")
    for row in rows:
        print(f"- {row['run_dir']}")


if __name__ == "__main__":
    raise SystemExit(main())
