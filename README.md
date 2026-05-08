Robot ROS Simulation (Ignition Gazebo + ROS 2)

Welcome to the robot_ros project. This package provides a complete, physics-accurate simulation environment for a custom 4-wheeled differential-drive robot, built using ROS 2 Humble and Ignition Gazebo (Fortress).

The robot is designed for applications such as autonomous navigation and badminton court line marking.

Features
Custom 4-Wheeled Robot Assembly
Accurately modeled using custom .STL meshes with properly aligned differential-drive kinematics.
Multiple Environments
The robot can be simulated in different worlds, including a complex roadmap environment (roadmap.world) and a structured sports court (badminton.world).
Closed-Loop PID Control
Includes a custom Python-based PID velocity controller that converts teleoperation commands into smooth and stable wheel velocities.
Full ROS 2 Integration
Provides complete TF tree broadcasting, odometry feedback, and LiDAR sensor integration.
Architecture and Working

This project is composed of several interconnected systems:

1. Physics and URDF (urdf/robot_ros.urdf)

The robot’s physical structure is defined using URDF. Primitive shapes such as cylinders and boxes are used for collision modeling to ensure efficient and stable simulation. Visual meshes (.STL) are aligned separately to maintain accurate appearance.

The system uses the following Gazebo plugins:

ignition::gazebo::systems::DiffDrive
Converts /cmd_vel commands into wheel velocities.
ignition::gazebo::systems::Sensors
Enables LiDAR functionality.
ignition::gazebo::systems::JointStatePublisher
Publishes joint states to build the ROS TF tree.
2. ROS–Gazebo Bridge (launch/badminton.launch.py)

Since ROS 2 and Ignition Gazebo use different communication systems, the ros_gz_bridge is used to connect them.

The launch file creates bidirectional bridges for:

Velocity commands (Twist)
Odometry data (Odometry)
LiDAR data (LaserScan)
Joint states (JointStates)
Transform and time data (TFMessage and Clock)
3. PID Velocity Controller (scripts/pid_velocity_controller.py)

Default Gazebo controllers produce unrealistic motion due to instantaneous velocity changes. To address this, a custom PID controller is implemented.

Input: Subscribes to /cmd_vel_raw (user commands)
Feedback: Subscribes to /model/robot_ros/odometry (actual robot velocity)
Output: Publishes smoothed /cmd_vel commands to Gazebo

This ensures gradual acceleration and stable motion.

Prerequisites

Ensure the following are installed:

Ubuntu 22.04
ROS 2 Humble
Ignition Gazebo Fortress (ign-gazebo6)
ros-humble-ros-gz
ros-humble-teleop-twist-keyboard
Getting Started
1. Build the Workspace
cd ~/robot_ws
colcon build --packages-select robot_ros --symlink-install
2. Source the Environment
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
3. Launch the Simulation

Badminton Court Environment:

ros2 launch robot_ros badminton.launch.py

Road Map Environment:

ros2 launch robot_ros roadmap.launch.py

This will launch Ignition Gazebo, spawn the robot, initialize ROS bridges, and start the PID controller.

Driving the Robot

Since a PID controller is used, commands must be sent to /cmd_vel_raw instead of /cmd_vel.

Open a new terminal and run:

source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
Controls
i → Move forward
, → Move backward
j → Turn left
l → Turn right
k → Stop
PID Tuning

To adjust the robot’s motion behavior:

Open config/pid_params.yaml

Modify the gains:

kp (Proportional gain)
ki (Integral gain)

Keep kp relatively low (for example, around 0.2) to avoid oscillations.

Restart the simulation to apply changes.
