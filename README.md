# Line Follower Robot
This repository contains the code and design files for a Line Follower Robot, an autonomous robot that follows a line using sensors. The project was developed as part of the Embedded Computing Architectures course for the Master's in Automation of Electrical and Computer Engineering and demonstrates fundamental concepts of control systems, sensor integration, and autonomous navigation.

## Overview
This repository contains the code for a **Line Follower Robot**, an autonomous robot that follows a line using sensors. The project was developed as part of the **Embedded Computing Architectures** course and demonstrates fundamental concepts of control systems, sensor integration, and autonomous navigation.

## Features
- Uses **Analog 5 Sensor Bar** for precise line detection.
- Controlled by a **Raspberry Pi Pico W**.
- Supports **differential drive** motor control using an **L298N Motor Driver**.
- Implements **PID (Proportional-Integral-Derivative) control** for optimized tracking.
- Includes obstacle detection functionality (sensor issues prevented full implementation).
- Modular and customizable codebase.

## Components
### Hardware
- **Raspberry Pi Pico W** (Microcontroller)
- **Motherboard** (For organizing components and connections)
- **DC motors (2x) with L298N Motor Driver**
- **Analog 5 Sensor Bar** (For line detection)
- **TOF Sensor** (For obstacle detection - issue encountered)
- **Li-Ion battery pack** (Power supply)
- **Cables and connectors**

### Software
- **C++ for Raspberry Pi Pico W**
- **VS Code with PlatformIO extension**

## Challenges and Issues Faced
- **Defective Distance Sensor (TOF Sensor):** This prevented the implementation of obstacle avoidance and maze-solving functionality.
- **Handling Line Interruptions:** Initially, the robot struggled in areas where the line was missing. A reactive strategy was implemented, making the robot rotate in place to relocate the line.
- **PID Tuning:** Finding the right balance for **KP** and **KD** was crucial for smooth tracking.

## Customization
- Modify the **PID controller** values for different track conditions.
- Adjust motor speeds and sensor sensitivity for optimal performance..

## Results
Despite challenges, the robot performed well, efficiently following lines with speed and accuracy. The optimized PID controller minimized oscillations and improved responsiveness. Future improvements include replacing the defective sensor and refining algorithms for handling missing lines.


