import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gazebo_cmd = LaunchConfiguration("gazebo_cmd")

    desc_pkg = get_package_share_directory("my_robot_description")
    bringup_pkg = get_package_share_directory("my_robot_bringup")

    xacro_file = os.path.join(desc_pkg, "urdf", "my_robot.urdf.xacro")
    controllers_file = os.path.join(bringup_pkg, "config", "controllers.yaml")

    robot_description = Command(["xacro ", xacro_file])

    # Publish TF from URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description}
        ],
        output="screen",
    )

    # ros2_control controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description},
            controllers_file,
        ],
        output="screen",
    )

    # Spawn robot into Gazebo
    # (if gazebo_ros spawn_entity.py available)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "my_bot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    # Load controllers
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    load_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Manually start Gazebo in another terminal for Fortress compatibility
            DeclareLaunchArgument(
                "gazebo_cmd",
                default_value="ign gazebo -r empty.sdf",
                description='Command you run for Gazebo (Fortress: "ign gazebo -r empty.sdf", newer: "gz sim -r empty.sdf")',
            ),
            robot_state_publisher,
            control_node,
            spawn_entity,
            load_jsb,
            load_diff,
        ]
    )
