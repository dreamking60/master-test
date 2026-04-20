# PPT Build Plan: Goal-Design-Result-Conclusion Structure

This slide plan is designed around the professor's feedback: every experiment should be presented as a research design, not just as a list of scripts or implementation work.

Use this repeated structure for each experiment:

```text
Goal -> Hypothesis -> Experiment Design -> Evidence/Measurement -> Result -> Conclusion
```

The presentation should answer four questions for every experiment:

1. What did I want to verify?
2. How did I design the experiment to verify it?
3. What result did I observe or measure?
4. What conclusion does that result support?

## Recommended Length

Use 13 to 16 slides.

Target timing:

- 8 minutes: 10 to 12 slides, merge some experiment result slides.
- 12 minutes: 13 to 15 slides.
- 15 minutes: 15 to 16 slides plus short demo or recorded evidence.

## Presentation Thesis

One sentence for the whole presentation:

```text
This project evaluates ROS2 TurtleBot3 command-channel security by designing controlled experiments that compare open ROS2 behavior, SROS2 access control, availability attacks, and network-layer MITM attacks in a migrated WSL + Docker environment.
```

Shorter version:

```text
The project shows what SROS2 protects, what it does not protect, and how those claims can be verified experimentally.
```

## Slide Deck Outline

### Slide 1: Title

Title:

```text
ROS2 Security Evaluation on TurtleBot3
```

Subtitle:

```text
Experiment Design for Command Injection, SROS2 Defense, DoS, and MITM
```

Include:

- Name.
- Course/project.
- Date.

Speaker message:

```text
My project is not only about running attacks. The main contribution is designing repeatable experiments to test ROS2 security claims under controlled conditions.
```

### Slide 2: Research Problem

Goal of this slide:

Explain the overall security problem.

Content:

```text
A ROS2 robot executes physical motion based on networked command messages.
If an attacker can publish, delay, drop, or modify those messages, a software communication problem becomes a physical safety problem.
```

Show a simple diagram:

```text
Controller -> ROS2/DDS -> /cmd_vel -> Gazebo TurtleBot3
```

Main question:

```text
Which attacks are possible in open ROS2, and which risks remain after SROS2 is enabled?
```

### Slide 3: Research Questions and Experiment Map

Use this table:

| Research Question | Experiment | Expected Security Meaning |
| --- | --- | --- |
| Can an unauthorized node inject robot commands? | Experiment 1: Open Injection | baseline vulnerability |
| Can SROS2 block unauthorized command publishers? | Experiment 2: SROS2 Defense | access-control protection |
| Can SROS2 still suffer availability degradation? | Experiment 3: DoS | security boundary |
| Can network MITM affect ROS2 traffic? | Experiment 4: ARP MITM | network-path risk |
| Can MITM rewrite SROS2-protected commands? | Experiment 5: SROS2 MITM | integrity protection vs availability |

Speaker message:

```text
Each experiment tests one specific security claim. The results build from simple open injection to stronger network-path attacks.
```

### Slide 4: Experimental Platform Design

Goal:

Explain why the environment was designed this way.

Design:

```text
WSL host: Gazebo GUI and ROS-Gazebo bridge
Docker controller: legitimate command publisher
Docker attacker: unauthorized publisher or network attacker
Docker robot-gateway: endpoint for MITM experiments
```

Diagram:

```text
Docker Controller / Attacker
        |
        v
     /cmd_vel_in
        |
        v
WSL cmd_vel_relay
        |
        v
     /cmd_vel
        |
        v
Gazebo TurtleBot3
```

Why this design:

- Gazebo needs reliable GUI support, so it runs on WSL host.
- Docker gives repeatable isolated roles for controller and attacker.
- The relay makes the command path stable and observable in WSL + Docker.

Speaker message:

```text
The platform itself was part of the project because the original three-VM setup was difficult to maintain. The new design keeps the same security roles but makes experiments repeatable.
```

### Slide 5: Experiment 1 Goal and Design - Open Command Injection

Goal:

```text
Verify whether an unauthorized ROS2 node can affect robot motion when SROS2 is disabled.
```

Hypothesis:

```text
If ROS2 has no authentication or topic-level permission, an attacker that joins the same ROS2 domain can publish to the command path and change robot behavior.
```

Design:

| Role | Location | Behavior |
| --- | --- | --- |
| Controller | Docker | publishes forward commands at 10 Hz |
| Attacker | Docker | publishes turn commands at higher rate |
| Robot | WSL/Gazebo | executes command stream through relay |

Measurement:

- Gazebo trajectory changes.
- `/cmd_vel_in` topic shows attacker/controller command competition.
- relay logs show commands reaching Gazebo path.

Demo command:

```bash
./scripts/demo/tmux_three_machine_demo.sh open
```

### Slide 6: Experiment 1 Result and Conclusion

Result:

```text
When the attacker starts publishing, the robot trajectory deviates from the normal controller path.
```

Evidence to show:

- screenshot/video: straight motion before attack, turning/deviation after attack.
- topic/log evidence: command messages relayed into Gazebo.

Conclusion:

```text
Open ROS2 command topics are unsafe for robot control because an unauthorized publisher can influence physical behavior.
```

Security meaning:

```text
This establishes the baseline risk that SROS2 is supposed to address.
```

### Slide 7: Experiment 2 Goal and Design - SROS2 Access Control

Goal:

```text
Verify whether SROS2 can block an unauthorized command publisher while allowing the legitimate controller.
```

Hypothesis:

```text
If the policy allows only `/controller` to publish `/cmd_vel_in`, then `/attacker` should fail to create or use a valid command publisher.
```

Design:

| Enclave | Permission |
| --- | --- |
| `/controller` | can publish `/cmd_vel_in` |
| `/gazebo` | can subscribe `/cmd_vel_in` and publish `/cmd_vel` |
| `/attacker` | cannot publish command topics |

Measurement:

- Controller still moves the robot.
- Attacker publisher creation fails or has no effect.
- FastDDS/SROS2 logs show permission denial.

Demo command:

```bash
./scripts/demo/tmux_sros2_defense_demo.sh
```

### Slide 8: Experiment 2 Result and Conclusion

Result:

```text
The secure controller can control the robot, while the attacker is blocked by SROS2 permissions.
```

Evidence to show:

```text
topic not found in allow rule
SROS2 policy blocked attacker publisher creation on /cmd_vel_in
```

Conclusion:

```text
SROS2 access control is effective against unauthorized command injection.
```

Security meaning:

```text
SROS2 changes the system from open publication to authenticated and permission-controlled publication.
```

Important caveat:

```text
This proves authorization protection, not complete protection against all network attacks.
```

### Slide 9: Experiment 3 Goal and Design - SROS2 DoS / Availability

Goal:

```text
Test whether SROS2-secured communication can still experience availability degradation under DDS/RTPS traffic pressure.
```

Hypothesis:

```text
Even if SROS2 blocks unauthorized publishers, high-rate UDP traffic toward DDS/RTPS ports can still consume resources or disturb command timing.
```

Design:

```text
Secure controller + secure Gazebo baseline
Attacker does not publish valid commands
Attacker floods discovered DDS/RTPS UDP ports
Measure controller stalls and relay watchdog events
```

Measurement:

- discovered DDS ports, such as `7400` and `7410-7423`.
- packet count and attacker CPU usage.
- controller publish-loop stalls.
- relay watchdog: missing `/cmd_vel_in` and zero `/cmd_vel` output.

Reference:

```text
docs/presentation/DOS_QA_CHEATSHEET.md
```

### Slide 10: Experiment 3 Result and Conclusion

Result summary:

```text
The attack did not bypass SROS2 authorization, but it caused short command-path interruptions.
```

Evidence:

```text
Sent 613,326 packets in 26.2s
avg 23,386 pkt/s
controller publish-loop stall around 3s
relay: No /cmd_vel_in for 2.93s; published zero /cmd_vel
```

Conclusion:

```text
SROS2 protects authentication and authorization, but it is not a complete availability defense.
```

Security meaning:

```text
Robot security needs both middleware security and network/resource protection.
```

### Slide 11: Experiment 4 Goal and Design - Open Network MITM

Goal:

```text
Verify whether an attacker can become a network-path MITM between controller and robot endpoints.
```

Hypothesis:

```text
If controller, robot, and attacker have separate L2 identities on the same Docker bridge, ARP poisoning can redirect traffic through the attacker.
```

Design:

```text
controller: 172.28.0.10
robot:      172.28.0.20
attacker:   172.28.0.50
```

Attack method:

```text
ARP poison both endpoints
Enable attacker IP forwarding
Apply tc netem delay/loss
Measure ping RTT and ROS2 command latency
```

Why Docker bridge:

```text
ARP spoofing requires distinct IP/MAC identities. Docker host networking cannot represent this attack model.
```

### Slide 12: Experiment 4 Result and Conclusion

Evidence:

```text
logs/experiments/03_network_mitm/recorded_20260419_120134/
```

Result table:

| Metric | Result |
| --- | ---: |
| Ping RTT average | `547.7 ms` |
| Ping RTT maximum | `2383.0 ms` |
| ROS2 command last-latency average | `286.3 ms` |
| ROS2 command max latency | `2335.0 ms` |
| ROS2 max command gap | `1418.9 ms` |

Conclusion:

```text
The Docker bridge lab successfully reproduced network-path MITM conditions, and the attacker could degrade ROS2 command timing.
```

Security meaning:

```text
Network-path control is different from publisher injection and requires separate network-level defenses.
```

### Slide 13: Experiment 5 Goal and Design - SROS2 MITM

Goal:

```text
Test what a network MITM can and cannot do when the ROS2 command traffic is protected by SROS2.
```

Hypothesis:

```text
SROS2 will not stop ARP spoofing itself, but it should prevent the attacker from validly rewriting protected DDS payloads.
The attacker should still be able to delay or drop packets.
```

Design:

```text
Secure controller enclave: /mitm_controller
Secure robot endpoint enclave: /mitm_robot
Attacker: ARP poison + delay/loss, no command-topic permission
```

Measurement:

- ARP tables show controller and robot poisoned to attacker MAC.
- qdisc shows `500ms` delay and `5%` loss.
- SROS2 subscriber latency and command gaps increase.
- No valid command rewrite is demonstrated.

### Slide 14: Experiment 5 Result and Conclusion

Evidence:

```text
logs/experiments/03_network_mitm/sros2_recorded_20260420_051959/
```

Result table:

| Metric | Result |
| --- | ---: |
| Ping RTT average | `537.7 ms` |
| Ping RTT maximum | `2673.0 ms` |
| SROS2 command last-latency average | `264.9 ms` |
| SROS2 command max latency | `2174.5 ms` |
| SROS2 max command gap | `2531.3 ms` |

Gazebo visual evidence:

| Metric | Result |
| --- | ---: |
| Average command receive rate | `8.37 commands/s` |
| Expected normal rate | `10 commands/s` |
| Maximum ROS command latency | `2996.1 ms` |
| Watchdog stop events | `13` |

Conclusion:

```text
SROS2 does not prevent the attacker from becoming a network MITM, but it prevents valid command payload rewriting. The remaining impact is availability degradation through delay/drop.
```

Security meaning:

```text
SROS2 protects integrity and authorization; network QoS, ARP protection, rate limiting, and monitoring are still needed for availability.
```

### Slide 15: Cross-Experiment Summary

Use this table as the main final summary:

| Experiment | Goal | Design | Verified Result | Conclusion |
| --- | --- | --- | --- | --- |
| Open Injection | test default ROS2 exposure | attacker publishes command topic | robot deviates | open ROS2 command path is unsafe |
| SROS2 Defense | test authorization | attacker lacks publish permission | publisher blocked | SROS2 blocks unauthorized command injection |
| SROS2 DoS | test availability boundary | UDP flood DDS ports | stalls/watchdog events | SROS2 is not full availability defense |
| Open MITM | test network-path control | ARP poison bridge endpoints | latency/gap increases | MITM topology reproduced |
| SROS2 MITM | test protected payload under MITM | ARP MITM + encrypted DDS | delay/drop but no valid rewrite | SROS2 protects integrity, not availability |

Speaker message:

```text
The experiments support a layered security conclusion: ROS2 security must be evaluated at application, middleware, and network layers.
```

### Slide 16: Conclusion and Future Work

Final conclusion:

```text
SROS2 is effective for command-topic authorization and payload integrity, but it does not eliminate network-layer availability attacks. A secure robot needs both DDS/SROS2 security and network-level protections.
```

Future work:

- SLAM/perception security: `/scan`, `/odom`, `/tf`, camera, map topics.
- Credential-compromise scenario: what if attacker steals a valid enclave key?
- Real TurtleBot3 hardware validation.
- Network defenses: ARP inspection, VLAN separation, firewall/rate limiting, QoS.

## How To Speak Each Experiment

Use this exact sentence pattern:

```text
The goal of this experiment was to verify [security claim].
To test it, I designed [controlled setup].
I measured [specific evidence].
The result was [observed data].
This supports the conclusion that [security meaning].
```

Example for SROS2 MITM:

```text
The goal was to verify whether SROS2-protected traffic can be modified by a network MITM.
To test it, I placed controller, robot endpoint, and attacker in a Docker bridge network and used ARP poisoning to route traffic through the attacker.
I measured ARP tables, qdisc delay/loss, and SROS2 command latency.
The result showed successful ARP MITM and increased latency, but no valid command rewrite.
This supports the conclusion that SROS2 protects payload integrity, while availability still needs network-level defense.
```

## Live Demo Strategy

Recommended live demo order:

1. Open injection: most visually clear.
2. SROS2 defense: show attacker blocked.
3. SROS2 MITM: show logs rather than relying only on visual trajectory.

Use recorded evidence for DoS and MITM if time is short.

Commands:

```bash
./scripts/demo/tmux_three_machine_demo.sh open
./scripts/demo/tmux_sros2_defense_demo.sh
./scripts/demo/tmux_sros2_gazebo_mitm_demo.sh
```

For MITM evidence, show:

```text
logs/experiments/03_network_mitm/sros2_recorded_20260420_051959/summary.md
logs/experiments/03_network_mitm/sros2_gazebo_udp_receiver.log
```

## Figures To Prepare

Prepare these visuals before building slides:

1. Original three-VM environment diagram.
2. Migrated WSL + Docker architecture diagram.
3. `/cmd_vel_in -> relay -> /cmd_vel -> Gazebo` data path.
4. Experiment design table for each experiment.
5. SROS2 policy/enclave permission diagram.
6. DoS evidence screenshot: packet count, controller stall, watchdog.
7. MITM topology diagram with IP/MAC roles.
8. SROS2 MITM result table showing delay/drop but no rewrite.
9. Final cross-experiment summary table.

## Backup Slides

Use backup slides for details if asked:

- Environment variables.
- Key files and scripts.
- Full SROS2 policy snippets.
- Why Docker host network cannot do ARP spoofing.
- Difference between DoS and DDoS.
- Difference between MITM tampering and SROS2 credential compromise.
- Why SROS2 does not stop ARP spoofing.
