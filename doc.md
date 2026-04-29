# 🧮 Mathematics & Code Architecture (`doc.md`)

This document dives deep into the mathematics and core algorithms powering the `robot_ros` project. It covers the physics of the differential drive, the mathematics of the PID controller, and the coordinate transformations used for the 3D meshes.

---

## 1. Differential Drive Kinematics

The robot utilizes a 4-wheel differential drive setup (skid-steer), which is mathematically modeled as a 2-wheel differential drive by the Ignition Gazebo plugin.

### 📐 Mathematics
Given:
*   $v$: Linear velocity of the robot center (m/s)
*   $\omega$: Angular velocity of the robot center (rad/s)
*   $L$: Wheel separation distance (track width) = `0.24 m`
*   $r$: Wheel radius = `0.05 m`

The target velocities for the left ($v_l$) and right ($v_r$) wheels are calculated as:
$$v_l = v - \frac{\omega \cdot L}{2}$$
$$v_r = v + \frac{\omega \cdot L}{2}$$

Gazebo converts these linear wheel speeds into angular joint commands (how fast the joints should spin in rad/s):
$$\omega_{left\_joints} = \frac{v_l}{r}$$
$$\omega_{right\_joints} = \frac{v_r}{r}$$

### 💻 Code Snippet: Gazebo Plugin URDF
This mathematical model is handled entirely by the Ignition DiffDrive plugin. We map the physical parameters directly into the URDF:

```xml
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
```

---

## 2. PID Velocity Control Algorithm

Because Gazebo's internal joint controllers apply infinite acceleration to reach target speeds, we implemented an outer-loop discrete PID controller. This calculates a smoothed velocity command to feed into Gazebo.

### 📐 Mathematics
At any given timestep $t$:
1.  **Error Calculation:**
    $$e(t) = v_{desired}(t) - v_{actual}(t)$$
2.  **Proportional Term:** 
    $$P(t) = K_p \cdot e(t)$$
3.  **Integral Term (with Anti-Windup):** 
    $$I(t) = \text{clamp}\left( I(t-\Delta t) + K_i \cdot e(t) \cdot \Delta t, \ -I_{max}, \ I_{max} \right)$$
4.  **Derivative Term (on Error):** 
    $$D(t) = K_d \cdot \frac{e(t) - e(t-\Delta t)}{\Delta t}$$
5.  **Output Command:**
    $$u(t) = P(t) + I(t) + D(t)$$

*Note:* Because this is a velocity controller wrapping around an internal velocity actuator, a high $K_p$ causes divergent oscillations. Therefore, the system relies heavily on the Integral term ($K_i$) for smooth acceleration.

### 💻 Code Snippet: Python PID Implementation
From `scripts/pid_velocity_controller.py`:

```python
def compute(self, setpoint: float, measurement: float) -> float:
    now = time.monotonic()
    dt = now - self._prev_time
    error = setpoint - measurement

    # 1. Proportional
    p_term = self.kp * error

    # 2. Integral with anti-windup clamping
    self._integral += error * dt
    self._integral = max(-self.integral_limit,
                         min(self.integral_limit, self._integral))
    i_term = self.ki * self._integral

    # 3. Derivative
    derivative = (error - self._prev_error) / dt
    d_term = self.kd * derivative

    output = p_term + i_term + d_term

    self._prev_error = error
    self._prev_time = now
    return output
```

---

## 3. Coordinate Frame Transformations (Visual Offsets)

During development, the imported `.STL` meshes for the wheels did not have their origins perfectly centered on the physical wheel axis. 

If we simply moved the physical `<joint>` origins to align the visuals, the DiffDrive kinematics (from Section 1) would explode because the mathematical layout of the wheels would no longer form a symmetric rectangle. 

### 📐 Mathematics
To solve this, we decoupled the **Physics Origin** from the **Visual Origin**.
Let:
*   $P_{joint}$: The mathematically perfect, symmetric origin for the physics engine.
*   $P_{mesh}$: The coordinate position where the `.STL` visually looks correct relative to the base.

The required transformation offset for the visual geometry relative to the joint is simply:
$$V_{offset} = P_{mesh} - P_{joint}$$

For example, for the front right wheel:
*   $P_{joint} = (0.33, \ -0.12, \ -0.0125)$
*   $P_{mesh} = (0.60, \ -0.145, \ -0.105)$
*   $V_{offset} = (0.27, \ -0.025, \ -0.0925)$

### 💻 Code Snippet: URDF Visual Offsets
By applying $V_{offset}$ to the `<visual>` tag, the physics engine spins the cylinder at the perfect location, but renders the 3D mesh slightly offset to match your aesthetic design.

```xml
  <!-- FRONT RIGHT WHEEL -->
  <link name="front_right_wheel">
    <visual>
      <!-- V_offset applied here to fix STL misalignment -->
      <origin xyz="0.27 -0.025 -0.0925" rpy="1.5708 0 1"/>
      <geometry>
        <mesh filename="package://robot_ros/meshes/front_right_wheel.STL"/>
      </geometry>
    </visual>
    <collision>
      <!-- Physics collision remains perfectly centered on the joint -->
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </collision>
  </link>

  <joint name="front_right_wheel_joint" type="continuous">
    <!-- P_joint: Symmetrical physical axis for DiffDrive -->
    <origin xyz="0.33 -0.12 -0.0125" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
```
