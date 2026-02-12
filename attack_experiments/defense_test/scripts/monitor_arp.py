#!/usr/bin/env python3
"""
ARP integrity monitor (defensive tool)

Goal:
  Detect suspicious ARP/neighbor table changes that often accompany MITM attempts
  (e.g., gateway IP suddenly resolves to a different MAC).

How it works:
  - Periodically snapshots `ip neigh` (Linux)
  - Tracks IP -> MAC mappings
  - Alerts on changes for watched IPs (gateway and/or explicit IPs)
  - Optionally writes/loads a baseline JSON file for reproducible experiments

This tool is intended for isolated lab/defense testing and incident triage.
It does NOT generate spoofed ARP packets or perform any attack actions.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional, Tuple


MAC_RE = re.compile(r"^[0-9a-fA-F]{2}(:[0-9a-fA-F]{2}){5}$")
IPV4_RE = re.compile(r"^\d{1,3}(\.\d{1,3}){3}$")


@dataclass(frozen=True)
class RouteInfo:
    gateway_ip: Optional[str]
    interface: Optional[str]


def now_ts() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def run(cmd: list[str], timeout: float = 2.0) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def get_default_route() -> RouteInfo:
    """
    Parse `ip route show default` output.
    Typical: "default via 192.168.1.1 dev ens33 proto dhcp metric 100"
    """
    try:
        p = run(["ip", "route", "show", "default"], timeout=2.0)
    except FileNotFoundError:
        return RouteInfo(gateway_ip=None, interface=None)
    except subprocess.TimeoutExpired:
        return RouteInfo(gateway_ip=None, interface=None)

    if p.returncode != 0:
        return RouteInfo(gateway_ip=None, interface=None)

    gw_ip = None
    iface = None
    for line in (p.stdout or "").splitlines():
        parts = line.strip().split()
        if not parts or parts[0] != "default":
            continue
        # Find "via <ip>" and "dev <iface>"
        for i, tok in enumerate(parts):
            if tok == "via" and i + 1 < len(parts):
                gw_ip = parts[i + 1]
            if tok == "dev" and i + 1 < len(parts):
                iface = parts[i + 1]
        break
    return RouteInfo(gateway_ip=gw_ip, interface=iface)


def parse_ip_neigh(stdout: str) -> Dict[str, str]:
    """
    Parse `ip neigh show` lines into {ip: mac}.
    Example:
      192.168.1.1 dev ens33 lladdr 00:11:22:33:44:55 REACHABLE
      192.168.1.10 dev ens33 lladdr aa:bb:cc:dd:ee:ff STALE
      192.168.1.123 dev ens33 FAILED
    We only keep entries that have a lladdr MAC.
    """
    mapping: Dict[str, str] = {}
    for line in stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        ip = parts[0] if parts else None
        if not ip or not IPV4_RE.match(ip):
            continue
        if "lladdr" not in parts:
            continue
        idx = parts.index("lladdr")
        if idx + 1 >= len(parts):
            continue
        mac = parts[idx + 1].lower()
        if not MAC_RE.match(mac):
            continue
        mapping[ip] = mac
    return mapping


def snapshot_neighbors(interface: Optional[str]) -> Dict[str, str]:
    cmd = ["ip", "neigh", "show"]
    if interface:
        cmd += ["dev", interface]
    p = run(cmd, timeout=2.0)
    if p.returncode != 0:
        return {}
    return parse_ip_neigh(p.stdout or "")


def load_baseline(path: str) -> Dict[str, str]:
    with open(path, "r", encoding="utf-8") as f:
        obj = json.load(f)
    if not isinstance(obj, dict):
        raise ValueError("Baseline JSON must be an object: {ip: mac}")
    out: Dict[str, str] = {}
    for k, v in obj.items():
        if not isinstance(k, str) or not isinstance(v, str):
            continue
        ip = k.strip()
        mac = v.strip().lower()
        if IPV4_RE.match(ip) and MAC_RE.match(mac):
            out[ip] = mac
    return out


def save_baseline(path: str, mapping: Dict[str, str]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(mapping, f, indent=2, sort_keys=True)
        f.write("\n")


def log(line: str, log_file: Optional[str]) -> None:
    out = f"[{now_ts()}] {line}"
    print(out, flush=True)
    if log_file:
        os.makedirs(os.path.dirname(log_file) or ".", exist_ok=True)
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(out + "\n")


def compute_collisions(mapping: Dict[str, str]) -> Dict[str, list[str]]:
    """
    Return {mac: [ip1, ip2, ...]} for MACs associated with multiple IPs.
    This can be normal in some environments, but is useful as a weak signal.
    """
    inv: Dict[str, list[str]] = {}
    for ip, mac in mapping.items():
        inv.setdefault(mac, []).append(ip)
    return {mac: ips for mac, ips in inv.items() if len(ips) >= 2}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Monitor ARP/neighbor table integrity (defensive). Linux only (requires `ip`)."
    )
    parser.add_argument(
        "--interface",
        type=str,
        default=None,
        help="Interface to monitor (default: detect from default route)",
    )
    parser.add_argument(
        "--watch-ip",
        type=str,
        nargs="*",
        default=[],
        help="Specific IPv4 addresses to watch (alerts on MAC changes)",
    )
    parser.add_argument(
        "--watch-gateway",
        action="store_true",
        help="Also watch the default gateway IP (recommended)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="Polling interval seconds (default: 2.0)",
    )
    parser.add_argument(
        "--baseline-in",
        type=str,
        default=None,
        help="Load baseline JSON mapping {ip: mac}. If omitted, baseline is taken at start.",
    )
    parser.add_argument(
        "--baseline-out",
        type=str,
        default=None,
        help="Write baseline mapping to this JSON file at startup.",
    )
    parser.add_argument(
        "--log-file",
        type=str,
        default=None,
        help="Append logs to this file (optional).",
    )
    parser.add_argument(
        "--print-collisions",
        action="store_true",
        help="Periodically print MAC collisions (weak signal).",
    )
    parser.add_argument(
        "--collisions-every",
        type=int,
        default=10,
        help="How many iterations between collision prints (default: 10).",
    )
    args = parser.parse_args()

    route = get_default_route()
    iface = (args.interface or route.interface or "").strip() or None
    gw_ip = route.gateway_ip if args.watch_gateway else None

    if iface is None:
        log(
            "ERROR: Could not determine interface. Provide --interface (Linux only).",
            args.log_file,
        )
        return 2

    watch: set[str] = set(ip.strip() for ip in args.watch_ip if ip.strip())
    if gw_ip and gw_ip.strip():
        watch.add(gw_ip.strip())

    # Baseline mapping
    try:
        if args.baseline_in:
            baseline = load_baseline(args.baseline_in)
        else:
            baseline = snapshot_neighbors(iface)
    except Exception as e:
        log(f"ERROR: Failed to load/take baseline: {e}", args.log_file)
        return 2

    if args.baseline_out:
        try:
            save_baseline(args.baseline_out, baseline)
            log(f"Baseline saved to {args.baseline_out}", args.log_file)
        except Exception as e:
            log(f"WARNING: Failed to save baseline: {e}", args.log_file)

    log(f"Monitoring interface: {iface}", args.log_file)
    if args.watch_gateway:
        log(f"Default gateway: {route.gateway_ip or 'unknown'}", args.log_file)
    if watch:
        log(f"Watched IPs: {', '.join(sorted(watch))}", args.log_file)
    else:
        log("Watched IPs: (none) — will only print collisions if enabled", args.log_file)

    # Show initial mappings for watched IPs (if present)
    for ip in sorted(watch):
        mac = baseline.get(ip)
        if mac:
            log(f"BASELINE {ip} -> {mac}", args.log_file)
        else:
            log(f"BASELINE {ip} -> (no entry yet)", args.log_file)

    last_seen = dict(baseline)
    it = 0
    try:
        while True:
            it += 1
            current = snapshot_neighbors(iface)

            # Alerts on watched IP mapping changes
            for ip in sorted(watch):
                prev_mac = last_seen.get(ip)
                curr_mac = current.get(ip)
                base_mac = baseline.get(ip)

                if curr_mac is None:
                    # Don't spam; only log transitions from known->unknown
                    if prev_mac is not None:
                        log(f"INFO {ip}: entry disappeared (was {prev_mac})", args.log_file)
                    continue

                if prev_mac is None:
                    log(f"INFO {ip}: entry appeared -> {curr_mac}", args.log_file)
                elif curr_mac != prev_mac:
                    # High-signal: mapping changed
                    hint = ""
                    if base_mac and curr_mac != base_mac:
                        hint = f" (baseline was {base_mac})"
                    log(f"ALERT {ip}: MAC changed {prev_mac} -> {curr_mac}{hint}", args.log_file)

            # Optional weak-signal: MAC collisions
            if args.print_collisions and (it % max(args.collisions_every, 1) == 0):
                collisions = compute_collisions(current)
                if collisions:
                    for mac, ips in sorted(collisions.items()):
                        log(f"NOTICE MAC collision {mac} <- {', '.join(sorted(ips))}", args.log_file)
                else:
                    log("NOTICE No MAC collisions in current neighbor table", args.log_file)

            last_seen = current
            time.sleep(max(args.interval, 0.2))
    except KeyboardInterrupt:
        log("Monitoring stopped.", args.log_file)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

