# Mid-Progress Report
Steven Chen - chensteven@wustl.edu

## 1. Introduction
Cyberphysical security is an essential component in the design and operation of modern systems, especially in the realms of robotics and autonomous vehicles. These systems rely heavily on interconnected sensors, actuators, and communication networks, making them vulnerable to various cyber threats. As technology advances, ensuring the integrity, confidentiality, and availability of these systems becomes increasingly vital. The stakes are high, as any security breach can lead not only to data loss or system malfunctions but also pose significant safety risks. Consequently, addressing and enhancing cyberphysical security measures is crucial for the development and deployment of reliable and resilient autonomous technologies. 

The project focuses on the security of TurtleBot3 using ROS. Its primary objective is to analyze and evaluate the security measures in TurtleBot3 with ROS. The project is organized into two main parts: 
1. Conducting a security analysis and evaluation of ROS combined with ORB-SLAM3 on TurtleBot3.
2. Developing secure components for the system.

My current analysis is on ROS2 with and without its security features. I have build experiment for Non-secure ROS2 and Secure ROS2 and to see what's the performance of the secure system of the ROS2 system. And how seucre version of ROS2 solve the possiblity attack from the non-secure system.

## 2. Progress
The current progress of the project is about environment setup and basic testing. For the first two months, I have set up an environment for the project and set up a future route for analysis and development. And for the next one month, I start to do some experiments on the security of the system.

I have done three experiments:
1. Non-secure ROS2 with Gazebo simulation /cmd_vel injection attack experiment
2. Secure ROS2 with Gazebo simulation /cmd_vel defense experiment
3. Secure ROS2 with Gazebo simulation /cmd_vel MITM attack experiment

### 2.1 Environment Setup
I have prepare three environments for the project. These environments are used for the experiments and development. As for better testing and experiment, I use virtual machine for experiments as a reference and then move to real world environment for final experiments.

1. The first environment is docker environment with ROS1 and orbslam3. The configuration is based on ubuntu 20.04 docker image with ros1 humble and orbslam3. The environment is used for the basic testing of the system. And also I found that it is very hard to configure such environment on my mac and for test. So I buld another two environments for work and test.

2. The second environment is ubuntu24 virtual machine with ROS2 humble. My finishing experiments are all on this environment. This environment is easy to install ROS2 package and also easy to install gazebo simulator for the experiments.

3. The thrid environment is real world environment with TurtleBot3 and ROS2 humble. This environment is used for the final experiments and development.

### 2.2 Experiments

#### 2.2.1 Non-secure ROS2 with Gazebo simulation /cmd_vel injection attack experiment

This experiment demonstrates the vulnerability of default ROS2 system without security features. The experiment is conducted on Gazebo simulation environment with TurtleBot3 robot model.

**Experiment Setup:**
1. Launch Gazebo simulation with empty world and TurtleBot3 robot
2. Start a normal controller node that publishes `/cmd_vel` commands at 10 Hz frequency with continuous forward movement pattern
3. Launch an attack node that attempts to inject malicious commands to the same `/cmd_vel` topic at 50 Hz frequency (5x higher than normal controller)
4. Record robot trajectory data during the experiment for analysis

**Attack Method:**
The attack uses frequency-based override strategy. The attacker publishes commands at much higher frequency (50 Hz) compared to the normal controller (10 Hz). This allows the attacker to fill the ROS2 message queue faster and dominate the topic, effectively overriding the legitimate control commands.

**Results:**
The experiment successfully demonstrates the vulnerability. During the attack period (5-20 seconds), the robot behavior changed significantly:
- Before attack: Robot moved straight forward at 0.169 m/s average velocity
- During attack: Robot turned left with 130.3° yaw change and angular velocity increased to 0.422 rad/s
- After attack: Robot returned to normal forward movement pattern

The trajectory data shows clear evidence of successful attack. The robot deviated from its expected straight path and followed the attacker's turn-left commands. This proves that default ROS2 topics are vulnerable to injection attacks when no access control or authentication is implemented.

**Key Findings:**
1. Higher frequency publishing (50 Hz vs 10 Hz) successfully overrides normal control
2. No authentication mechanism prevents unauthorized nodes from publishing to critical topics
3. ROS2 message queue behavior allows last-write-wins scenario, favoring higher frequency publishers
4. The attack is effective and hard to detect without monitoring tools

#### 2.2.2 Secure ROS2 with Gazebo simulation /cmd_vel defense experiment

This experiment tests the security mechanisms provided by SROS2 (Secure ROS2) to defend against command injection attacks. The experiment evaluates whether SROS2 can effectively prevent unauthorized access to ROS2 topics.

**Experiment Setup:**
1. Setup Certificate Authority (CA) for the secure ROS2 environment
2. Generate certificates for legitimate nodes (robot controller and Gazebo simulation)
3. Configure security policies that define access permissions for each node
4. Test legitimate access with proper certificates
5. Attempt unauthorized attacks without certificates or with wrong certificates

**Security Configuration:**
- Certificate-based authentication: All nodes must have valid certificates signed by the CA
- Access control policies: Only authorized nodes can publish to `/cmd_vel` topic
- Encryption: All messages are encrypted using TLS/DTLS
- Message integrity: Messages are signed to prevent tampering

**Test Scenarios:**
1. **Legitimate Access Test**: Robot controller with valid certificate and proper permissions can successfully control the robot
2. **Unauthorized Attack Test**: Attack node without any security credentials cannot discover ROS2 nodes or publish messages
3. **Wrong Credentials Test**: Attack node with certificate but without permission in security policy cannot publish to `/cmd_vel` topic

**Results:**
The SROS2 security mechanism successfully prevents unauthorized attacks:
- Legitimate nodes with proper certificates work normally
- Unauthorized nodes without certificates cannot join the ROS2 network
- Nodes with certificates but wrong permissions are blocked by security policies
- Attack attempts fail because the attacker cannot authenticate or get proper permissions

**Key Findings:**
1. SROS2 provides effective protection against unauthorized access
2. Certificate-based authentication prevents unauthorized nodes from joining the network
3. Security policies enforce fine-grained access control
4. Encryption protects message content from network sniffing
5. However, if legitimate credentials are stolen, the security can be bypassed

#### 2.2.3 Secure ROS2 with Gazebo simulation /cmd_vel MITM attack experiment

This experiment tests Man-in-the-Middle (MITM) attack on ROS2 system, both with and without SROS2 security. The experiment uses a three-machine setup where the attacker machine acts as a router to intercept and modify traffic between the robot and legitimate controller.

The three machine are on the same network segment. And they are:
1. Robot machine (target)
2. Controller machine (legitimate operator)
3. Attacker machine (MITM router)

**Experiment Setup:**
1. Configure three machines: Robot machine (target), Controller machine (legitimate operator), and Attacker machine (MITM router)
2. Setup attacker machine as router with IP forwarding enabled
3. Configure robot and controller to route traffic through the attacker machine
4. Launch interception and modification script on attacker machine
5. Test MITM attack on ROS2 communication

**MITM Attack Method:**
The attacker machine is positioned as a router in the network path. All traffic between robot and controller flows through the attacker. The attacker attempts to:
- Intercept all DDS traffic between robot and controller
- Read message contents
- Modify `/cmd_vel` messages before forwarding
- Log all communication for analysis

**Results:**
The MITM attack successfully intercepted and modified the `/cmd_vel` commands. The attacker was able to read the legitimate control commands from the controller machine and modify them before forwarding to the robot. The robot received and executed the modified commands, demonstrating that the MITM attack was successful in hijacking the robot control.

**Key Findings:**
1. MITM attack successfully intercepted all DDS traffic between robot and controller
2. The attacker successfully modified `/cmd_vel` messages before forwarding
3. Robot executed the modified commands, proving the attack effectiveness
4. Network layer routing enables MITM attacks when attacker controls the network path
5. The attack demonstrates the vulnerability of ROS2 communication without proper security measures


## 3. Future Work

The current MITM attack experiment focuses on application level attacks, where the attacker modifies ROS2 messages at the application layer. For future work, I plan to conduct MITM attacks at lower network layers to demonstrate more fundamental vulnerabilities in the communication stack.

The next experiment will target network layer or transport layer attacks, such as:
1. Packet-level interception and modification at the network layer
2. DDS protocol manipulation at the transport layer
3. Lower-level attack methods that bypass application layer defenses

After successfully completing the lower-level MITM attack experiments in the simulation environment, I plan to migrate all experiments to the real TurtleBot3 robot. This will validate the attack methods in real-world scenarios and provide more realistic security assessment results. The real robot experiments will help identify additional security concerns that may not appear in simulation environments.

After completing the attack experiments on the real TurtleBot3 robot, I plan to develop and implement defense mechanisms to protect against these attacks. 

The defense mechanisms will be tested against the attack methods developed in the experiments to evaluate their effectiveness and performance impact on the robot system.