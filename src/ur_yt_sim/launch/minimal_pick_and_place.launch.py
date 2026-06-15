"""
Minimal Pick-and-Place Launch File
Uses Gazebo's built-in physics, no ros2_control to avoid crashing
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- share dirs ---
    uryt_share     = get_package_share_directory("ur_yt_sim")
    robotiq_share  = get_package_share_directory("robotiq_description")
    ur_share       = get_package_share_directory("ur_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    world_file = os.path.join(uryt_share, 'worlds', 'world2.world')

    # --- ENV Gazebo ---
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=":".join(["/usr/share/gazebo-11", uryt_share, robotiq_share, ur_share])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=":".join([
            os.path.join(uryt_share, "models"),
            os.path.join(robotiq_share, "models"),
            os.path.expanduser("~/.gazebo/models")
        ])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=":".join([
            "/opt/ros/humble/lib",
            os.path.normpath(os.path.join(uryt_share, "..", "..", "lib")),
            os.path.normpath(os.path.join(uryt_share, "..", "ros2_linkattacher", "lib")),
        ])
    ))

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
        launch_arguments={"use_sim_time": "true", "gui": "false", "paused": "false", "world": world_file}.items()
    )
    ld.add_action(gazebo)

    # --- Robot Description Command (with fake hardware to avoid ros2_control) ---
    # This prevents gazebo_ros2_control plugin from being loaded, which was causing crashes
    robot_description_content = Command([
        "ros2 run xacro xacro ",
        os.path.join(ur_share, "urdf", "ur.urdf.xacro"),
        " name:=ur5 ur_type:=ur5 sim_gazebo:=false use_fake_hardware:=true headless_mode:=true"
    ])

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": True
            }
        ],
        output="screen",
    )
    ld.add_action(robot_state_publisher)

    # --- Spawn Robot ---
    spawn_ur5 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "cobot",
            "-topic", "robot_description",
        ],
        output="screen",
    )
    ld.add_action(TimerAction(period=3.0, actions=[spawn_ur5]))

    return ld
