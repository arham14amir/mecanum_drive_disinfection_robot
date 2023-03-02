import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ====================================================
    # 1. HARDWARE & SENSORS CONFIGURATION
    # ====================================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("mecanum_drive"), "description", "robot.urdf.xacro"])
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([FindPackageShare("mecanum_drive"), "config", "mecanum_controllers.yaml"])
    ekf_config_path = PathJoinSubstitution([FindPackageShare("mecanum_drive"), "config", "ekf.yaml"])

    # --- Core Hardware Nodes ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[('/mecanum_drive_controller/odometry', '/odom_unfiltered'),
                    ('/mecanum_drive_controller/tf_odometry', '/tf_null')],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # --- Spawners & Relays ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
    )

    cmd_vel_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/mecanum_drive_controller/reference_unstamped']
    )

    # --- Sensors & Localization ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'])]),
        launch_arguments={'frame_id': 'laser_frame'}.items()
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('mpu6050driver'), 'launch', 'mpu6050driver_launch.py'])])
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # ====================================================
    # 2. NAV2 CONFIGURATION
    # ====================================================

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])]),
        launch_arguments={
            'use_sim_time': 'False',
            'map': '/root/mecanum_drive_2/maps/department.yaml',
            'params_file': '/root/mecanum_drive_2/config/nav2_params.yaml'
        }.items()
    )

    # ====================================================
    # 3. TIMERS (ESP Bug Fix & Nav2 Delay)
    # ====================================================
    
    # T=4s: Start Base Controllers (Fixes ESP Auto-Reset)
    delay_joint_state = TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner])
    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[robot_controller_spawner])
    )

    # T=8s: Start Nav2 (Gives EKF and Base time to publish transforms)
    delay_nav2 = TimerAction(period=8.0, actions=[nav2_launch])

    return LaunchDescription([
        # Phase 1: Hardware & Sensors (Immediate)
        control_node,
        robot_state_pub_node,
        lidar_launch,
        imu_launch,  
        ekf_node,    
        cmd_vel_relay_node,

        # Phase 2: Delayed Base Controllers
        delay_joint_state,
        delay_robot_controller,

        # Phase 3: Delayed Autonomy Stack
        delay_nav2
    ])