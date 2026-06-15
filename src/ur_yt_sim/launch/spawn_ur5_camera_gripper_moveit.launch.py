from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- share dirs ---
    uryt_share     = get_package_share_directory("ur_yt_sim")
    robotiq_share  = get_package_share_directory("robotiq_description")
    ur_share       = get_package_share_directory("ur_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    world_file = os.path.join(get_package_share_directory('ur_yt_sim'), 'worlds', 'world2.world')

    # --- ENV Gazebo ---
    gazebo_model_database_uri = os.environ.get("GAZEBO_MODEL_DATABASE_URI", "")
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI",
        value=gazebo_model_database_uri
    ))

    existing_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    resource_paths = ["/usr/share/gazebo-11", uryt_share, robotiq_share, ur_share]
    if existing_resource_path:
        resource_paths.append(existing_resource_path)
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=":".join(resource_paths)
    ))

    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    model_paths = [
        os.path.join(uryt_share, "models"),
        os.path.join(robotiq_share, "models"),
        os.path.expanduser("~/.gazebo/models"),
        os.path.dirname(ur_share),
        os.path.dirname(uryt_share),
        os.path.dirname(robotiq_share)
    ]
    if existing_model_path:
        model_paths.append(existing_model_path)
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=":".join(model_paths)
    ))

    existing_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    plugin_paths = [
        "/opt/ros/humble/lib",
        os.path.normpath(os.path.join(uryt_share, "..", "..", "lib")),
    ]
    if existing_plugin_path:
        plugin_paths.append(existing_plugin_path)
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=":".join(plugin_paths)
    ))

    # --- args ---
    with_rviz     = DeclareLaunchArgument("with_rviz", default_value="true")
    with_octomap  = DeclareLaunchArgument("with_octomap", default_value="false")
    gui_arg       = DeclareLaunchArgument("gui", default_value="true")
    x_arg = DeclareLaunchArgument("x", default_value="0")
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")
    ld.add_action(with_rviz); ld.add_action(with_octomap); ld.add_action(gui_arg)
    ld.add_action(x_arg); ld.add_action(y_arg); ld.add_action(z_arg)

    # --- MoveIt config ---
    joint_controllers_file = os.path.join(uryt_share, "config", "ur5_controllers_gripper_140.yaml")
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur5_camera_gripper_moveit_config")
        .robot_description(
            file_path="config/ur.urdf.xacro",
            mappings={
                "ur_type": "ur5",
                "sim_gazebo": "true",
                "sim_ignition": "false",
                "use_fake_hardware": "false",
                "simulation_controllers": joint_controllers_file,
                "initial_positions_file": os.path.join(uryt_share, "config", "initial_positions.yaml"),
            },
        )
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
        launch_arguments={"use_sim_time":"true", "gui":LaunchConfiguration("gui"), "pause":"false", "world": world_file}.items()
    )
    ld.add_action(gazebo)

    # --- RSP ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )
    ld.add_action(robot_state_publisher)

    # --- Spawn ---
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity","cobot",
            "-topic","robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
    ld.add_action(TimerAction(period=3.0, actions=[spawn]))

    # --- Controller Manager ---
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, joint_controllers_file, {"use_sim_time": True}],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    # ld.add_action(controller_manager_node)

    jsb  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"],
                parameters=[{"use_sim_time": True}], output="screen")
    arm  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_trajectory_controller","--controller-manager","/controller_manager"],
                parameters=[{"use_sim_time": True}], output="screen")
    grip = Node(package="controller_manager", executable="spawner",
                arguments=["gripper_position_controller","--controller-manager","/controller_manager"],
                parameters=[{"use_sim_time": True}], output="screen")

    ld.add_action(RegisterEventHandler(
        OnProcessStart(target_action=spawn, on_start=[
            TimerAction(period=2.0, actions=[jsb]),
            TimerAction(period=3.0, actions=[arm, grip]),
        ])
    ))

    # Define flat kinematics config for correct ROS 2 parameter parsing
    kinematics_config = {
        "robot_description_kinematics": {
            "ur5_manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.005,
                "kinematics_solver_attempts": 3,
            }
        }
    }

    rviz_cfg = os.path.join(get_package_share_directory("ur5_camera_gripper_moveit_config"),
                            "config", "moveit.rviz")
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            kinematics_config,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz"))
    )
    ld.add_action(rviz)

    mg_params = moveit_config.to_dict()
    
    # Construct moveit_simple_controller_manager dict
    controllers_config = {}
    if "controller_names" in mg_params:
        controllers_config["controller_names"] = mg_params["controller_names"]
    if "joint_trajectory_controller" in mg_params:
        controllers_config["joint_trajectory_controller"] = mg_params["joint_trajectory_controller"]
    if "gripper_position_controller" in mg_params:
        controllers_config["gripper_position_controller"] = mg_params["gripper_position_controller"]

    mg_params.update({
        "use_sim_time": True,
        "default_planning_pipeline": "ompl",
        "ompl.planning_plugin": "ompl_interface/OMPLPlanner",
        "ompl.request_adapters": (
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints"
        ),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": controllers_config,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    })
    mg_params.update(kinematics_config)

    move_group_with_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params],
        arguments=["--ros-args","--log-level","info"],
        condition=IfCondition(LaunchConfiguration("with_octomap")),   # << NEW
    )

    mg_params_no_sensors = dict(mg_params)
    mg_params_no_sensors.pop("sensors", None)

    move_group_no_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params_no_sensors],
        arguments=["--ros-args","--log-level","info"],
        condition=UnlessCondition(LaunchConfiguration("with_octomap")),
    )

    

    ld.add_action(move_group_with_octomap)
    ld.add_action(move_group_no_octomap)

    # --- Gazebo Joint Mirror ---
    joint_mirror = Node(
        package="ur_yt_sim",
        executable="gazebo_joint_mirror",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    # ld.add_action(joint_mirror)

    return ld
