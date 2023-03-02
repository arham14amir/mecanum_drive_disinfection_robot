import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, TimerAction, ExecuteProcess
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
    # 2. NAV2 & AUTONOMY NODES
    # ====================================================

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])]),
        launch_arguments={
            'use_sim_time': 'False',
            'map': '/root/mecanum_drive_2/maps/department.yaml',
            'params_file': '/root/mecanum_drive_2/config/nav2_params.yaml'
        }.items()
    )

    safety_speaker_node = Node(
        package=None,
        executable='python3',
        arguments=['/root/mecanum_drive_2/safety_speaker.py'],
        output='screen'
    )

    opennav_node = Node(
        package='opennav_coverage',
        executable='opennav_coverage',
        name='coverage_server',
        output='screen',
        arguments=['--ros-args', '--params-file', '/root/mecanum_drive_2/config/mecanum_coverage.yaml']
    )

    # PERMANENT FIX: Poll until coverage_server is ready, then configure+activate
    opennav_lifecycle = ExecuteProcess(
        cmd=['bash', '/root/mecanum_drive_2/wait_and_configure.sh'],
        output='screen'
    )

    run_cover_room = ExecuteProcess(
        cmd=['python3', '/root/mecanum_drive_2/cover_room.py'],
        output='screen'
    )

    # ====================================================
    # 3. CASCADING TIMERS & EVENT TRIGGERS
    # ====================================================

    delay_joint_state = TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner])
    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[robot_controller_spawner])
    )

    delay_nav2 = TimerAction(period=15.0, actions=[nav2_launch])

    start_autonomy_stack = TimerAction(
        period=15.0,
        actions=[
            opennav_node,
            safety_speaker_node,
            TimerAction(period=3.0, actions=[opennav_lifecycle]),
            TimerAction(period=12.0, actions=[run_cover_room])
        ]
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        lidar_launch,
        imu_launch,
        ekf_node,
        cmd_vel_relay_node,
        delay_joint_state,
        delay_robot_controller,
        delay_nav2,
        start_autonomy_stack
    ])
