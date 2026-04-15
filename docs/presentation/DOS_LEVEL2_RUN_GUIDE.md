# SROS2 DoS Level 2 Run Guide

## Purpose

Level 2 keeps the Level 1 experiment design, but increases UDP flood intensity in three controlled steps.

The goal is to test whether stronger DDS/RTPS UDP flood traffic makes the short command-path interruptions more repeatable or more severe.

## Scripts

Run these one at a time:

```bash
./scripts/experiments/run_sros2_dos_level2_low.sh
./scripts/experiments/run_sros2_dos_level2_medium.sh
./scripts/experiments/run_sros2_dos_level2_high.sh
```

Each script writes evidence under:

```text
logs/experiments/log_driven_validation/<timestamp>/sros2_dos/
```

Each run includes:

- `experiment_config.env`
- `dos_attack.log`
- `controller_session.log`
- `resource_monitor.log`
- `test.log`
- `test_markers.log`
- `trajectory_summary.txt`

## Intensity Levels

| Script | Label | Payload | Threads per DDS port | Duration | Purpose |
| --- | --- | ---: | ---: | ---: | --- |
| `run_sros2_dos_level2_low.sh` | `level2_low` | 1200 bytes | 6 | 35s | Check whether doubling Level 1 intensity repeats the stall. |
| `run_sros2_dos_level2_medium.sh` | `level2_medium` | 1400 bytes | 10 | 45s | Increase packet size and concurrency. |
| `run_sros2_dos_level2_high.sh` | `level2_high` | 1400 bytes | 16 | 60s | Stress test. Use only after low/medium complete safely. |

Defaults can be overridden:

```bash
DOS_THREADS_PER_PORT=8 ATTACK_SECONDS=30 ./scripts/experiments/run_sros2_dos_level2_medium.sh
```

## What To Watch During Runs

Important logs:

```bash
tail -n 120 logs/experiments/log_driven_validation/*/sros2_dos/dos_attack.log
tail -n 160 logs/experiments/log_driven_validation/*/sros2_dos/controller_session.log
tail -n 160 logs/experiments/log_driven_validation/*/sros2_dos/test_markers.log
tail -n 200 logs/experiments/log_driven_validation/*/sros2_dos/resource_monitor.log
```

Evidence of impact:

- `controller_session.log`: `published=` counter stops increasing at 10Hz.
- `test.log` / `test_markers.log`: `No /cmd_vel_in for ...; published zero /cmd_vel`.
- `resource_monitor.log`: attacker CPU rises sharply; controller CPU may rise or stall.
- `trajectory_summary.txt`: useful only if the SROS2-aware recorder is fixed.

## Safety Notes

The high level can consume several CPU cores. If the system becomes too slow:

```bash
pkill -f dos_attack_sros2.py
sudo ./scripts/wsl_docker/down.sh
pkill -f turtlebot3_empty_world_custom_bridge.launch.py
```

The relay watchdog should stop stale motion by publishing zero `/cmd_vel` when `/cmd_vel_in` is missing.

## How To Send Results For Analysis

After all three runs, send the latest directories:

```bash
find logs/experiments/log_driven_validation -maxdepth 2 -type d -name sros2_dos -printf '%T@ %p\n' | sort -n | tail -3
```

Then I will compare:

- packet count and pkt/s
- attacker CPU
- controller publish stalls
- relay watchdog events
- whether impact grows from low to medium to high

You can also generate a quick local summary:

```bash
python3 scripts/experiments/summarize_dos_runs.py --latest 3
```
