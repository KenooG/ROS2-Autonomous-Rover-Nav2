# 🚀 AMRS: Autonomous Mars Rover System

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=flat-square&logo=ros)
![C++](https://img.shields.io/badge/C%2B%2B-17-green?style=flat-square&logo=c%2B%2B)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![Gazebo](https://img.shields.io/badge/Simulation-Gazebo%20Harmonic-orange?style=flat-square)

An advanced R&D simulation environment for autonomous Martian exploration. This project demonstrates a complete integration of the **Nav2 stack**, **asynchronous state machines**, and **Computer Vision** for real-time mission decision-making.

---

## 📋 Table of Contents
* [Mission Overview](#-mission-overview--decision-flow)
* [System Flow](#-system-flow)
* [Key Features](#-key-features)
* [Installation](#-installation)
* [How to Run](#-how-to-run)
* [What I Learned](#-what-i-learned)

---

## 🎯 Mission Overview & Decision Flow
The rover is tasked with navigating a hazardous Martian terrain to deliver priority resources. Unlike static pathfollowers, AMRS evaluates cargo on-the-fly:

1.  **Navigation:** The rover moves toward the first cargo checkpoint.
2.  **Perception:** Using the onboard camera, the system performs OCR.
3.  **Branching Logic:**
    * ✅ **Detected "FOOD":** High-priority trigger. Overrides current mission, cancels searching, and plots the fastest path to the Base Station.
    * ⚠️ **Detected "WASTE":** Hazard trigger. The rover re-routes to evaluate the second container before finalizing the delivery.

---

## 🔄 System Flow
A modular data pipeline ensuring low-latency response:
* **Perception:** LiDAR → `slam_toolbox` → Occupancy Grid Map
* **Navigation:** Map + Goal → `Nav2 Planner` → Dynamic Path
* **Vision AI:** Camera Stream → `cv_bridge` → OpenCV OCR Node
* **Logic:** OCR Result → Mission Control (C++) → Goal Preemption

---

## 🎥 Mission Demonstrations

| Scenario A: Priority Resource (Food) | Scenario B: Hazard Handling (Waste) |
| :--- | :--- |
| **[▶️ Watch Demo](https://youtu.be/FRYcUNDp-Kg)** | **[▶️ Watch Demo](https://www.youtube.com/watch?v=xDR68K2qFZU)** |
| *Rover detects Food and re-routes to base.* | *Rover detects Waste and re-evaluates second box.* |

---

## 📐 System Architecture
![ROS 2 Architecture](ros_architecture.svg)

---

## 🛠️ Key Features
* **Dynamic Goal Preemption:** Real-time path overriding based on asynchronous vision triggers.
* **SLAM Integration:** Real-time mapping and localization using `slam_toolbox`.
* **Hybrid Architecture:** High-performance C++ core for control logic + Python for rapid AI prototyping.
* **Custom Environment:** High-fidelity Gazebo world with Martian textures and physics.

---

## ⚙️ Installation

### Prerequisites
* Ubuntu 24.04 (or 22.04)
* ROS 2 Jazzy (or Humble)
* Gazebo Harmonic
* OpenCV & Tesseract OCR (`sudo apt install tesseract-ocr`)

### Build
```bash
# Clone the repository
cd ~/ros2_ws/src
git clone [https://github.com/KenooG/ROS2-Autonomous-Rover-Nav2.git](https://github.com/KenooG/ROS2-Autonomous-Rover-Nav2.git)

# Install dependencies and build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 How to Run

1.  **Start OCR Vision Node:**
    `ros2 run my_robot_description ocr_detector.py`
2.  **Launch Simulation:**
    `ros2 launch my_robot_description amrs_rover_launch.py`
3.  **Launch Navigation & SLAM:**
    `ros2 launch my_robot_description mars.mission.launch.py`
4.  **Run Mission Control:**
    `ros2 run my_robot_description mission_control_node`

### Triggering the autonomous sequence:
Open a new terminal and publish the final delivery coordinates:
```bash
ros2 topic pub --once /rover_goal rover_interfaces/msg/RoverGoal "{target_x: 20.0, target_y: -15.0}"
```

---

## 📌 What I Learned
* **Action Clients:** Utilized the `Nav2` Action Client (`NavigateToPose`) to asynchronously send and update goals without blocking the main thread.
* **Costmap Configuration:** Tuned inflation layers to balance between collision safety and the need to get close to objects for OCR scanning.
* **Asynchronous C++:** Implemented non-blocking ROS 2 Service calls and Wall Timers to ensure a responsive, real-time State Machine.
