# 🤖 Robot ROS Simulation (Ignition Gazebo + ROS 2)

Welcome to the `robot_ros` project! This package provides a complete, physics-accurate simulation environment for a custom 4-wheeled differential-drive robot, built using **ROS 2 Humble** and **Ignition Gazebo (Fortress)**. 

The robot is designed for tasks such as autonomous navigation and badminton court line-marking.

---

## ✨ Features

- **Custom 4-Wheeled Robot Assembly**: Accurately modeled using custom `.STL` meshes with perfectly aligned differential-drive kinematics.
- **Multiple Environments**: Simulate the robot in different worlds, including a complex road map (`roadmap.world`) and a sports court (`badminton.world`).
- **Closed-Loop PID Control**: Features a custom, highly-tuned Python PID velocity controller that smoothly translates teleop commands into stable wheel velocities.
- **Full ROS 2 Integration**: Complete TF tree broadcasting, Odometry feedback, and LiDAR sensor integration out-of-the-box.

---

## 🏗️ Architecture & How It Works

This project is built on several interconnected systems working together:

### 1. The Physics & URDF (`urdf/robot_ros.urdf`)
The robot's physical structure is defined here. We use primitive shapes (`cylinder`, `box`) for the physics engine collision meshes to ensure the simulation is extremely fast and stable (no physics "explosions"). The visual meshes (`.STL`) are offset dynamically to match the user's desired aesthetic look. 
It uses three core Gazebo plugins:
- `ignition::gazebo::systems::DiffDrive`: Converts `/cmd_vel` into wheel joint velocities.
- `ignition::gazebo::systems::Sensors`: Powers the robot's LiDAR.
- `ignition::gazebo::systems::JointStatePublisher`: Broadcasts the wheel joint states to build the ROS TF tree.

### 2. The Bridge (`launch/badminton.launch.py`)
Because Ignition Gazebo and ROS 2 use entirely different messaging systems, we use `ros_gz_bridge` to connect them. The launch file automatically creates bidirectional bridges for:
- `Twist` (Velocity commands)
- `Odometry` (Robot position/velocity)
- `LaserScan` (LiDAR data)
- `JointStates` (Wheel rotations)
- `TFMessage` & `Clock` (Time and transforms)

### 3. PID Velocity Controller (`scripts/pid_velocity_controller.py`)
Standard Gazebo velocity controllers are incredibly aggressive. If you command `0.5 m/s`, the robot tries to reach it instantly. To make the movement fluid and realistic, we use a custom PID Node.
- **Input:** Subscribes to `/cmd_vel_raw` (from your keyboard/joystick)
- **Feedback:** Subscribes to `/model/robot_ros/odometry` (actual speed from Gazebo)
- **Output:** Publishes a smoothed, corrected `/cmd_vel` to Gazebo.

---

## 🛠️ Prerequisites

Make sure your system has the following installed:
- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Ignition Gazebo Fortress** (`ign-gazebo6`)
- `ros-humble-ros-gz` (ROS-Ignition bridge packages)
- `ros-humble-teleop-twist-keyboard`

---

## 🚀 Getting Started

### 1. Build the Workspace
To build the project and ensure all custom Python scripts (like the PID controller) and `.STL` resources are linked correctly, navigate to your workspace and build:

```bash
cd ~/robot_ws
colcon build --packages-select robot_ros --symlink-install
```

### 2. Source the Environment
Always source your ROS 2 environment and your local workspace before running commands:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

### 3. Launch the Simulation
You can choose which world you want to simulate by running the respective launch file.

**For the Badminton Court:**
```bash
ros2 launch robot_ros badminton.launch.py
```

**For the Road Map:**
```bash
ros2 launch robot_ros roadmap.launch.py
```

This will automatically open Ignition Gazebo, spawn the robot, start the ROS bridges, and launch the PID velocity controller!

---

## 🎮 Driving the Robot

Because we use a custom PID controller to smooth the robot's movement, you cannot publish directly to `/cmd_vel` like a standard robot. Instead, you publish your desired velocity to `/cmd_vel_raw`. 

Open a **new terminal**, source ROS 2, and use the standard keyboard teleop node, remapping the topic:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
```

**Controls:**
- `i`: Move forward
- `,`: Move backward
- `j`: Turn left
- `l`: Turn right
- `k`: Stop

---

## 🎛️ Tuning the Robot's Handling

If you feel the robot accelerates too slowly, or you want to make it snappier, you don't need to touch the code! All tuning is handled via a configuration file.

1. Open `config/pid_params.yaml`
2. Adjust the `kp` (Proportional) and `ki` (Integral) gains. 
   - *Note: Keep `kp` very low (e.g., `0.2`) to prevent physics oscillation.*
3. Restart the launch file. 

Happy Simulating! 🚀
