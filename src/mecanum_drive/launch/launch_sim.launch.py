import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Get Package Paths
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mecanum_drive"), "description", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mecanum_drive"),
            "config",
            "mecanum_controllers.yaml",
        ]
    )

    # 2. Controller Manager (The Brain)
    # This replaces the Gazebo plugin. It talks to your ESP32 driver.
    # 2. Controller Manager (The Brain)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        # REMAP ADDED HERE
        remappings=[
            ('/mecanum_drive_controller/odometry', '/odom'),
            ('/mecanum_drive_controller/tf_odometry', '/tf')
        ],
        output="screen",
    )

    # 3. Robot State Publisher (The Body Transform)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 4. Spawners (The Activators)
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

    # 5. Delay controller spawning until the manager is ready
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])