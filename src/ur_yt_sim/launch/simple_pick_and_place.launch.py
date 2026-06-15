"""
Simple Pick-and-Place Launch File
Launches Gazebo with UR5 + Robotiq Gripper (without MoveIt)
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
        launch_arguments={"use_sim_time": "true", "gui": "true", "paused": "false", "world": world_file}.items()
    )
    ld.add_action(gazebo)

    # --- Robot Description Command ---
    robot_description_content = Command([
        "ros2 run xacro xacro ",
        os.path.join(ur_share, "urdf", "ur.urdf.xacro"),
        " name:=ur5 ur_type:=ur5 sim_gazebo:=false use_fake_hardware:=true"
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

    # --- Controllers ---
    jsb = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    arm = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    ld.add_action(RegisterEventHandler(
        OnProcessStart(target_action=spawn_ur5, on_start=[
            TimerAction(period=2.0, actions=[jsb, arm]),
        ])
    ))

    return ld
