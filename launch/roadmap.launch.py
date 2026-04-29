import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('robot_ros')          # …/share/robot_ros
    pkg_parent = os.path.dirname(pkg)                       # …/share/
    urdf_path  = os.path.join(pkg, 'urdf', 'robot_ros.urdf')
    world_path = os.path.join(pkg, 'world', 'roadmap.world')
    models_dir = os.path.join(pkg, 'models')
    pid_params = os.path.join(pkg, 'config', 'pid_params.yaml')

    # Source-tree equivalents (useful with --symlink-install)
    src_root = os.path.realpath(
        os.path.join(os.path.dirname(__file__), '..'))     # …/share/robot_ros
    src_models = os.path.join(src_root, 'models')
    src_parent = os.path.dirname(src_root)

    # ── Resource path ──────────────────────────────────────────────────────
    # Ignition resolves  model://robot_ros/...  by scanning these dirs for a
    # sub-dir named "robot_ros".  We must include the *parent* of the package
    # dir (i.e. share/) so that "robot_ros" is found inside it.
    resource_dirs = [models_dir, pkg, pkg_parent]
    if src_root != pkg:
        resource_dirs += [src_models, src_root, src_parent]
    resource_path = ':'.join(d for d in resource_dirs if os.path.isdir(d))

    with open(urdf_path, 'r') as fh:
        robot_description = fh.read()

    # ── Environment variables ──────────────────────────────────────────────
    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=resource_path)
    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path)
    set_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=resource_path)

    # ── Launch arguments ───────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    # ── Ignition Gazebo ────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '-v', '4',
            '-r',
            '--render-engine', 'ogre',
            world_path,
        ],
        output='screen',
        additional_env={
            'IGN_GAZEBO_RESOURCE_PATH': resource_path,
            'GZ_SIM_RESOURCE_PATH':     resource_path,
            'GAZEBO_MODEL_PATH':         resource_path,
        },
    )

    # ── Robot State Publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    # ── Spawn robot after Gazebo has initialised ───────────────────────────
    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-name',  'robot_ros',
                    '-topic', 'robot_description',
                    '-x',     '0.0',
                    '-y',     '0.0',
                    '-z',     '0.1',
                    '-Y',     '0.0',
                ],
            ),
        ],
    )

    # ── ROS ↔ Ignition bridge ──────────────────────────────────────────────
    # /cmd_vel_raw is NOT bridged — it stays in ROS and is consumed by the
    # PID node, which then publishes the corrected /cmd_vel to Ignition.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Drive — corrected /cmd_vel goes to Ignition DiffDrive
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            # LiDAR
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            # Odometry — used by PID node as feedback
            '/model/robot_ros/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # Joint states — feeds robot_state_publisher TF tree
            '/world/roadmap/model/robot_ros/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
            # TF
            '/model/robot_ros/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
    )

    # ── PID Velocity Controller ────────────────────────────────────────────
    # Reads /cmd_vel_raw  (from teleop), reads /model/robot_ros/odometry,
    # publishes corrected /cmd_vel  (→ bridge → Ignition DiffDrive).
    # Launch after Gazebo+robot are up so odometry is available quickly.
    pid_controller = TimerAction(
        period=10.0,   # start after robot is spawned
        actions=[
            Node(
                package='robot_ros',
                executable='pid_velocity_controller',
                name='pid_velocity_controller',
                output='screen',
                parameters=[pid_params],
            ),
        ],
    )

    return LaunchDescription([
        # Environment first
        set_gz_resource,
        set_ign_resource,
        set_gazebo_model,
        # Args
        use_sim_time_arg,
        # Processes / Nodes
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_robot,
        pid_controller,
    ])
