#!/usr/bin/env python3
"""
PID Velocity Controller for robot_ros (Ignition Fortress / ROS 2 Humble).

Control chain:
  [teleop]  -->  /cmd_vel_raw  -->  [THIS NODE]  -->  /cmd_vel  -->  [DiffDrive Plugin in Gazebo]

Subscribes:
  /cmd_vel_raw               (geometry_msgs/Twist)  — desired velocity from teleop
  /model/robot_ros/odometry  (nav_msgs/Odometry)    — actual velocity from Gazebo

Publishes:
  /cmd_vel                   (geometry_msgs/Twist)  — PID-corrected velocity

PID gains and limits are read from ROS parameters (set via pid_params.yaml).
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ─────────────────────────────────────────────────────────────────────────────
class PIDController:
    """Simple, standalone PID with anti-windup clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float = 2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time: float | None = None

    def compute(self, setpoint: float, measurement: float) -> float:
        now = time.monotonic()

        if self._prev_time is None:
            # First call — proportional only
            self._prev_time = now
            error = setpoint - measurement
            self._prev_error = error
            return self.kp * error

        dt = now - self._prev_time
        if dt <= 1e-6:
            return self.kp * self._prev_error  # avoid div-by-zero

        error = setpoint - measurement

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))

        # Derivative (on measurement, not error, to avoid derivative kick)
        derivative = (error - self._prev_error) / dt

        output = (self.kp * error
                  + self.ki * self._integral
                  + self.kd * derivative)

        self._prev_error = error
        self._prev_time = now
        return output

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None


# ─────────────────────────────────────────────────────────────────────────────
class PIDVelocityController(Node):

    def __init__(self):
        super().__init__('pid_velocity_controller')

        # ── Declare Parameters ──────────────────────────────────────────────
        self.declare_parameter('linear.kp',            1.5)
        self.declare_parameter('linear.ki',            0.3)
        self.declare_parameter('linear.kd',            0.05)
        self.declare_parameter('linear.integral_limit', 2.0)
        self.declare_parameter('linear.max_output',    2.5)

        self.declare_parameter('angular.kp',            2.5)
        self.declare_parameter('angular.ki',            0.2)
        self.declare_parameter('angular.kd',            0.08)
        self.declare_parameter('angular.integral_limit', 2.0)
        self.declare_parameter('angular.max_output',   2.5)

        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('deadband', 0.01)  # below this = treat as zero

        # ── Read Parameters ─────────────────────────────────────────────────
        lkp  = self.get_parameter('linear.kp').value
        lki  = self.get_parameter('linear.ki').value
        lkd  = self.get_parameter('linear.kd').value
        llim = self.get_parameter('linear.integral_limit').value
        self._max_lin = self.get_parameter('linear.max_output').value

        akp  = self.get_parameter('angular.kp').value
        aki  = self.get_parameter('angular.ki').value
        akd  = self.get_parameter('angular.kd').value
        alim = self.get_parameter('angular.integral_limit').value
        self._max_ang = self.get_parameter('angular.max_output').value

        rate = self.get_parameter('control_rate_hz').value
        self._deadband = self.get_parameter('deadband').value

        # ── PID instances ───────────────────────────────────────────────────
        self._linear_pid  = PIDController(lkp, lki, lkd, llim)
        self._angular_pid = PIDController(akp, aki, akd, alim)

        # ── State ───────────────────────────────────────────────────────────
        self._desired_linear  = 0.0
        self._desired_angular = 0.0
        self._actual_linear   = 0.0
        self._actual_angular  = 0.0
        self._odom_received   = False

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel_raw',
                                 self._cmd_callback, 10)
        self.create_subscription(Odometry, '/model/robot_ros/odometry',
                                 self._odom_callback, 10)

        # ── Publisher ───────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Control timer ───────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('─' * 60)
        self.get_logger().info('  PID Velocity Controller started')
        self.get_logger().info(f'  Linear  PID  Kp={lkp}  Ki={lki}  Kd={lkd}')
        self.get_logger().info(f'  Angular PID  Kp={akp}  Ki={aki}  Kd={akd}')
        self.get_logger().info(f'  Control rate : {rate} Hz')
        self.get_logger().info(f'  Input topic  : /cmd_vel_raw')
        self.get_logger().info(f'  Output topic : /cmd_vel')
        self.get_logger().info('─' * 60)
        self.get_logger().info(
            'Waiting for odometry … (robot must be spawned in Gazebo)')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Twist) -> None:
        self._desired_linear  = msg.linear.x
        self._desired_angular = msg.angular.z

    def _odom_callback(self, msg: Odometry) -> None:
        self._actual_linear   = msg.twist.twist.linear.x
        self._actual_angular  = msg.twist.twist.angular.z
        if not self._odom_received:
            self._odom_received = True
            self.get_logger().info('Odometry received — PID loop active.')

    # ── Control loop ─────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        out = Twist()

        # If no desire to move, send zero and reset integrators
        if (abs(self._desired_linear)  < self._deadband and
                abs(self._desired_angular) < self._deadband):
            self._linear_pid.reset()
            self._angular_pid.reset()
            self._pub.publish(out)
            return

        # If odometry not yet received, pass desired through directly
        # (open-loop fallback so the robot can still be commanded)
        if not self._odom_received:
            out.linear.x  = self._desired_linear
            out.angular.z = self._desired_angular
            self._pub.publish(out)
            return

        # PID computation
        lin = self._linear_pid.compute(self._desired_linear,
                                       self._actual_linear)
        ang = self._angular_pid.compute(self._desired_angular,
                                        self._actual_angular)

        # Clamp to max allowed outputs
        out.linear.x  = max(-self._max_lin, min(self._max_lin,  lin))
        out.angular.z = max(-self._max_ang, min(self._max_ang, ang))

        self._pub.publish(out)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PIDVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
