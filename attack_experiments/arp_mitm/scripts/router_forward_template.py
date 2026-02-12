#!/usr/bin/env python3
"""路由转发型 MITM 实验脚本模板。

该模板用于记录“攻击机先配置成合法路由/网关，再将控制端与机器人端流量转发”的实验。
所有真正的转发、抓包、ROS2 操作逻辑都保留为空白 TODO，由你在隔离环境中自行实现。
"""

import argparse
import json
import pathlib
import subprocess
import sys
import time
from typing import Optional


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Router-forwarding MITM template",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--iface-in", required=True, help="与控制端相连的接口")
    parser.add_argument("--iface-out", required=True, help="与机器人相连的接口")
    parser.add_argument("--controller-ip", required=True)
    parser.add_argument("--target-ip", required=True)
    parser.add_argument(
        "--log-dir",
        default="logs/arp_mitm",
        help="保存 JSON/文本日志的目录",
    )
    parser.add_argument(
        "--enable-forward",
        action="store_true",
        help="在脚本运行期间临时开启 net.ipv4.ip_forward",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印步骤，不执行任何系统命令",
    )
    return parser.parse_args()


def ensure_log_dir(path: pathlib.Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def log_event(log_path: pathlib.Path, event: dict) -> None:
    payload = {"ts": time.time(), **event}
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=False) + "\n")


def run_cmd(cmd: list[str], dry_run: bool) -> subprocess.CompletedProcess:
    print("[CMD]", " ".join(cmd))
    if dry_run:
        class Dummy:
            returncode = 0
            stdout = ""
            stderr = ""
        return Dummy()  # type: ignore[return-value]
    return subprocess.run(cmd, check=False, capture_output=True, text=True)


def toggle_ip_forward(enable: bool, dry_run: bool) -> None:
    value = "1" if enable else "0"
    run_cmd(["sysctl", "-w", f"net.ipv4.ip_forward={value}"], dry_run=dry_run)


def setup_routing(args: argparse.Namespace, log_path: pathlib.Path) -> None:
    if args.enable_forward:
        toggle_ip_forward(True, args.dry_run)
        log_event(log_path, {"action": "enable_ip_forward", "value": 1})

    # TODO: 在此添加 iptables/nftables/iptables-nft 规则，或使用 firewalld
    # 例如：run_cmd(["iptables", "-t", "nat", "-A", ...], args.dry_run)
    log_event(log_path, {"action": "configure_nat", "status": "TODO"})


def teardown_routing(args: argparse.Namespace, log_path: pathlib.Path) -> None:
    # TODO: 在这里移除你添加的防火墙或路由规则
    log_event(log_path, {"action": "cleanup_rules", "status": "TODO"})

    if args.enable_forward:
        toggle_ip_forward(False, args.dry_run)
        log_event(log_path, {"action": "enable_ip_forward", "value": 0})


def monitor_loop(args: argparse.Namespace, log_path: pathlib.Path) -> None:
    print("[INFO] 进入监控循环 (TODO: 在此实现 sniff / 日志记录 / ROS2 检查)")
    # TODO: 可在此处调用 scapy.sniff、tcpdump 或 ROS2 CLI 进行观察
    # while True:
    #     ...
    log_event(log_path, {"action": "monitor_placeholder"})


def main() -> None:
    args = parse_args()
    log_dir = pathlib.Path(args.log_dir)
    ensure_log_dir(log_dir)
    log_path = log_dir / f"router_forward_{int(time.time())}.json"

    print("=== Router Forwarding MITM Template ===")
    print("控制端接口:", args.iface_in)
    print("机器人接口:", args.iface_out)
    print("控制端 IP:", args.controller_ip)
    print("机器人 IP:", args.target_ip)
    print("日志文件:", log_path)

    if args.dry_run:
        print("Dry-run 模式，仅打印计划步骤")

    try:
        setup_routing(args, log_path)
        monitor_loop(args, log_path)
    finally:
        teardown_routing(args, log_path)
        print("[INFO] 已执行清理")


if __name__ == "__main__":
    sys.exit(main())
