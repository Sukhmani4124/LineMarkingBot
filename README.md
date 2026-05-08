# Robot ROS Simulation (Ignition Gazebo + ROS 2)

This project provides a physics-accurate simulation environment for a custom 4-wheeled differential-drive robot using ROS 2 Humble and Ignition Gazebo (Fortress).

The robot is designed for applications such as autonomous navigation and badminton court line marking.

---

## Features

- **Custom 4-Wheeled Robot**
  - Built using `.STL` meshes with aligned differential-drive kinematics

- **Multiple Simulation Environments**
  - `badminton.world` (structured court)
  - `roadmap.world` (complex navigation)

- **Closed-Loop PID Control**
  - Smooth and stable motion using a custom Python PID controller

- **Full ROS 2 Integration**
  - TF tree, odometry, and LiDAR sensor support

---

## Architecture Overview

The system consists of three main components:

### 1. Robot Model (URDF)

- Defines links, joints, and physical properties
- Uses simplified collision shapes for stable simulation
- Includes Gazebo plugins:
  - DiffDrive
  - Sensors (LiDAR)
  - Joint State Publisher

---

### 2. ROS–Gazebo Bridge

Connects ROS 2 and Ignition Gazebo using `ros_gz_bridge`.

Bridged topics include:

- `/cmd_vel` → velocity commands  
- `/odom` → odometry  
- `/scan` → LiDAR data  
- `/joint_states` → wheel states  
- `/tf` and `/clock`  

---

### 3. PID Velocity Controller

Custom ROS 2 node to smooth robot motion.

- Input: `/cmd_vel_raw`
- Feedback: `/odom`
- Output: `/cmd_vel`

Prevents unrealistic instantaneous acceleration.

---

## Prerequisites

Make sure you have:

- Ubuntu 22.04  
- ROS 2 Humble  
- Ignition Gazebo Fortress (`ign-gazebo6`)  
- `ros-humble-ros-gz`  
- `ros-humble-teleop-twist-keyboard`  

---

## Setup Instructions

### 1. Build Workspace

```bash
cd ~/robot_ws
colcon build --packages-select robot_ros --symlink-install
```

---

### 2. Source Environment

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

---

### 3. Launch Simulation

#### Badminton Court

```bash
ros2 launch robot_ros badminton.launch.py
```

#### Road Map

```bash
ros2 launch robot_ros roadmap.launch.py
```

This launches:
- Ignition Gazebo
- Robot model
- ROS bridge
- PID controller

---

## Teleoperation

Use keyboard control via `/cmd_vel_raw`.

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
```

### Controls

- `i` → forward  
- `,` → backward  
- `j` → left  
- `l` → right  
- `k` → stop  

---

## PID Tuning

Edit:

```
config/pid_params.yaml
```

Adjust:
- `kp` (keep low to avoid oscillations)
- `ki`

Restart simulation after changes.

---

## Project Structure

```
robot_ros/
├── urdf/
├── launch/
├── scripts/
├── config/
├── meshes/
├── worlds/
```

---

## Future Work

- SLAM integration  
- Autonomous navigation (Nav2)  
- Real-world deployment  
- Multi-robot coordination  

---

## Authors

- Sukhmani Kaur  
- Akshon Choudhary  
- Aarush Gupta  
- Garv Talwar  

---
