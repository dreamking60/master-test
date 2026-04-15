#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path


def load_rows(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as handle:
        for row in csv.DictReader(handle):
            rows.append({
                't': float(row['relative_time']),
                'x': float(row['x']),
                'y': float(row['y']),
                'yaw': float(row['yaw']),
                'linear_x': float(row['linear_x']),
                'angular_z': float(row['angular_z']),
            })
    return rows


def slice_rows(rows, start, end):
    return [row for row in rows if start <= row['t'] < end]


def path_distance(rows):
    total = 0.0
    for prev, cur in zip(rows, rows[1:]):
        total += math.hypot(cur['x'] - prev['x'], cur['y'] - prev['y'])
    return total


def max_gap(rows):
    if len(rows) < 2:
        return 0.0
    return max(cur['t'] - prev['t'] for prev, cur in zip(rows, rows[1:]))


def mean(values):
    return sum(values) / len(values) if values else 0.0


def summarize_period(name, rows, start, end):
    period = slice_rows(rows, start, end)
    if not period:
        return {
            'name': name,
            'start': start,
            'end': end,
            'samples': 0,
        }

    return {
        'name': name,
        'start': start,
        'end': end,
        'samples': len(period),
        'duration': period[-1]['t'] - period[0]['t'] if len(period) > 1 else 0.0,
        'distance_m': path_distance(period),
        'displacement_m': math.hypot(period[-1]['x'] - period[0]['x'], period[-1]['y'] - period[0]['y']),
        'yaw_change_rad': period[-1]['yaw'] - period[0]['yaw'],
        'avg_linear_x': mean([row['linear_x'] for row in period]),
        'avg_abs_angular_z': mean([abs(row['angular_z']) for row in period]),
        'max_sample_gap_s': max_gap(period),
        'x_range': (min(row['x'] for row in period), max(row['x'] for row in period)),
        'y_range': (min(row['y'] for row in period), max(row['y'] for row in period)),
    }


def format_period(summary):
    if summary['samples'] == 0:
        return [
            f"{summary['name']}: no samples in {summary['start']:.1f}s-{summary['end']:.1f}s",
        ]

    return [
        f"{summary['name']} ({summary['start']:.1f}s-{summary['end']:.1f}s)",
        f"  samples: {summary['samples']}",
        f"  duration_s: {summary['duration']:.3f}",
        f"  distance_m: {summary['distance_m']:.3f}",
        f"  displacement_m: {summary['displacement_m']:.3f}",
        f"  yaw_change_rad: {summary['yaw_change_rad']:.3f}",
        f"  avg_linear_x: {summary['avg_linear_x']:.3f}",
        f"  avg_abs_angular_z: {summary['avg_abs_angular_z']:.3f}",
        f"  max_sample_gap_s: {summary['max_sample_gap_s']:.3f}",
        f"  x_range: [{summary['x_range'][0]:.3f}, {summary['x_range'][1]:.3f}]",
        f"  y_range: [{summary['y_range'][0]:.3f}, {summary['y_range'][1]:.3f}]",
    ]


def classify(label, before, during):
    if before['samples'] == 0 or during['samples'] == 0:
        return "insufficient trajectory samples for classification"

    angular_delta = during['avg_abs_angular_z'] - before['avg_abs_angular_z']
    gap_ratio = during['max_sample_gap_s'] / max(before['max_sample_gap_s'], 0.001)
    distance_ratio = during['distance_m'] / max(before['distance_m'], 0.001)

    if label == 'open-injection':
        if angular_delta > 0.15:
            return "open injection likely affected motion: during-attack angular velocity increased"
        return "open injection effect not obvious from trajectory metrics"

    if label == 'sros2-dos':
        if during['max_sample_gap_s'] > 0.5 or gap_ratio > 3.0:
            return "DoS may have affected telemetry/control timing: during-attack sample gaps increased"
        if distance_ratio < 0.5:
            return "DoS may have affected motion: during-attack distance dropped sharply"
        return "no obvious DoS impact in trajectory metrics; record as negative result for this variant"

    return "classification not configured for this label"


def main():
    parser = argparse.ArgumentParser(description="Summarize TurtleBot3 trajectory CSV by experiment phases.")
    parser.add_argument("csv_file")
    parser.add_argument("--label", choices=["open-injection", "sros2-dos"], required=True)
    parser.add_argument("--baseline-seconds", type=float, required=True)
    parser.add_argument("--attack-seconds", type=float, required=True)
    parser.add_argument("--after-seconds", type=float, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rows = load_rows(args.csv_file)
    if not rows:
        Path(args.output).write_text("No trajectory samples recorded.\n", encoding='utf-8')
        return 2

    baseline_end = args.baseline_seconds
    attack_end = baseline_end + args.attack_seconds
    after_end = attack_end + args.after_seconds

    before = summarize_period("baseline", rows, 0.0, baseline_end)
    during = summarize_period("attack", rows, baseline_end, attack_end)
    after = summarize_period("after", rows, attack_end, after_end)

    lines = [
        f"Trajectory summary: {args.label}",
        f"csv: {args.csv_file}",
        f"total_samples: {len(rows)}",
        f"total_duration_s: {rows[-1]['t']:.3f}",
        "",
    ]
    for section in (before, during, after):
        lines.extend(format_period(section))
        lines.append("")

    lines.extend([
        "classification:",
        f"  {classify(args.label, before, during)}",
    ])

    Path(args.output).write_text("\n".join(lines) + "\n", encoding='utf-8')
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
