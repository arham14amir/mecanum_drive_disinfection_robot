# 🧼 Mecanum Drive Disinfection Robot

> Autonomous mecanum-wheeled mobile robot that navigates an indoor environment and performs area disinfection, built on **ROS 2**.

<p align="left">
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/Gazebo-Classic-FF6600?logo=gazebo&logoColor=white" />
  <img src="https://img.shields.io/badge/C++-00599C?logo=cplusplus&logoColor=white" />
  <img src="https://img.shields.io/badge/License-MIT-green" />
</p>

## Overview
A four-wheel **mecanum-drive** robot capable of **omnidirectional motion**, autonomous navigation, and coverage-based disinfection of a target area. Simulated in Gazebo with a full ROS 2 control and navigation stack.

## ✨ Features
- **Omnidirectional mecanum drive** — moves in any direction / rotates in place via `ros2_control`
- **Autonomous navigation** — mapping, localization, and path planning with **Nav2**
- **Coverage planning** for full-area disinfection
- **Teleoperation** support for manual control
- **Gazebo simulation** with a custom world and robot URDF

## 🧱 Tech Stack
`ROS 2` · `Gazebo` · `Nav2` · `ros2_control` · `C++` · `Python` · `URDF/Xacro` · `RViz`

## 📦 Requirements
- Ubuntu 22.04 + ROS 2 Humble
- `sudo apt install ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers`

## 🚀 Getting Started
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/arham14amir/mecanum_drive_disinfection_robot.git
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch <your_pkg> <your_launch>.launch.py
```

## 🕹️ Usage
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# or send a "Nav2 Goal" from RViz
```2026

## 📸 Demo
<img width="1600" height="1204" alt="WhatsApp Image 2026-07-16 at 12 54 19 AM" src="https://github.com/user-attachments/assets/55750d0d-9478-4574-8b8d-e3ba549595b3" />


## 👤 Author
**Muhammad Arham** — [GitHub](https://github.com/arham14amir) · [LinkedIn](https://www.linkedin.com/in/arhamamir)

## 📄 License
MIT
