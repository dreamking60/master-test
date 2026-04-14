# Experiment 04: SLAM Security and Data Integrity

## Purpose

Extend the security analysis from robot command topics to perception and mapping data used by SLAM.

This is future work and should be presented as the next research direction after the command-channel experiments are stable.

## Research Question

If an attacker can influence sensor, odometry, transform, or map-related topics, how does that affect localization and mapping reliability?

## Candidate Attack Surfaces

| Topic / Data | Risk |
| --- | --- |
| `/scan` | false obstacles, missing obstacles, distorted ranges |
| `/odom` | incorrect robot motion estimate |
| `/tf` | inconsistent coordinate frames |
| `/map` | corrupted occupancy grid |
| camera topics | ORB-SLAM3 feature corruption or tracking loss |

## Suggested Experiment Sequence

1. Establish normal SLAM baseline and record map quality.
2. Inject small perturbations into `/scan` or `/odom`.
3. Measure localization drift and map distortion.
4. Repeat with SROS2 permissions that restrict who can publish sensor and transform topics.
5. Compare mapping quality and attack success.

## Metrics

- Localization drift.
- Map consistency.
- Tracking loss count.
- False obstacle rate.
- Time to recovery after attack stops.

## Presentation Points

- Command-channel security protects actuation.
- SLAM security protects perception and world model integrity.
- A robot can be unsafe even if `/cmd_vel` is protected, if the attacker can corrupt localization or mapping inputs.
- This direction connects the current ROS2 security work to the original TurtleBot3 + ORB-SLAM3 project goal.

## Report Paragraph

Future work will extend the security evaluation from command injection to SLAM data integrity. The robot's behavior depends not only on velocity commands but also on perception and localization topics such as laser scans, odometry, transforms, and maps. Attacks against these inputs can degrade mapping quality or cause incorrect navigation decisions. The next stage of the project should evaluate how SROS2 permissions and additional validation mechanisms can protect SLAM-related topics.

