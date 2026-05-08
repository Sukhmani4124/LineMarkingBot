Mathematics and Code Architecture (doc.md)

This document explains the mathematical models and core algorithms used in the robot_ros project. It covers differential drive kinematics, PID control, and coordinate transformations for mesh alignment.

1. Differential Drive Kinematics

The robot uses a 4-wheel differential drive (skid-steer) configuration. For modeling purposes, this is treated as a 2-wheel differential drive system.

Mathematics

Let:

v = linear velocity of robot (m/s)
ω = angular velocity (rad/s)
L = wheel separation = 0.24 m
r = wheel radius = 0.05 m
Wheel Velocities
v_l = v - (ω * L / 2)
v_r = v + (ω * L / 2)
Angular Velocities
ω_left  = v_l / r
ω_right = v_r / r
URDF Plugin
<plugin
  filename="ignition-gazebo-diff-drive-system"
  name="ignition::gazebo::systems::DiffDrive">
  <left_joint>back_left_wheel_joint</left_joint>
  <left_joint>front_left_wheel_joint</left_joint>
  <right_joint>back_right_wheel_joint</right_joint>
  <right_joint>front_right_wheel_joint</right_joint>
  <wheel_separation>0.24</wheel_separation>
  <wheel_radius>0.05</wheel_radius>
  <odom_publish_frequency>30</odom_publish_frequency>
  <topic>cmd_vel</topic>
</plugin>
2. PID Velocity Control Algorithm

Default Gazebo controllers apply instantaneous acceleration. To make motion realistic, a discrete PID controller is used.

Mathematics

Error

e(t) = v_desired - v_actual

Proportional

P = Kp * e

Integral (with limit)

I = clamp(I + Ki * e * dt)

Derivative

D = Kd * (change in error / dt)

Output

u = P + I + D

Note: Keep Kp low to avoid oscillations.

Python Implementation
def compute(self, setpoint, measurement):
    now = time.monotonic()
    dt = now - self._prev_time
    error = setpoint - measurement

    # Proportional
    p_term = self.kp * error

    # Integral with limit
    self._integral += error * dt
    self._integral = max(-self.integral_limit,
                         min(self.integral_limit, self._integral))
    i_term = self.ki * self._integral

    # Derivative
    derivative = (error - self._prev_error) / dt
    d_term = self.kd * derivative

    output = p_term + i_term + d_term

    self._prev_error = error
    self._prev_time = now
    return output
3. Coordinate Frame Transformations (Visual Offsets)

The STL meshes were not aligned with the wheel axes. Changing joint positions would break kinematics, so physics and visuals were separated.

Concept
P_joint = physical joint position
P_mesh = visual mesh position
Offset
V_offset = P_mesh - P_joint
Example
P_joint = (0.33, -0.12, -0.0125)
P_mesh  = (0.60, -0.145, -0.105)

V_offset = (0.27, -0.025, -0.0925)
URDF Implementation
<link name="front_right_wheel">
  <visual>
    <origin xyz="0.27 -0.025 -0.0925" rpy="1.5708 0 1"/>
    <geometry>
      <mesh filename="package://robot_ros/meshes/front_right_wheel.STL"/>
    </geometry>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.03"/>
    </geometry>
  </collision>
</link>

<joint name="front_right_wheel_joint" type="continuous">
  <origin xyz="0.33 -0.12 -0.0125" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
