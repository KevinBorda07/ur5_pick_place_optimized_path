import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # --- MoveIt config ---
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur5_camera_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Pick and Place Node with explicit OMPL and request adapters
    moveit_params = moveit_config.to_dict()
    
    # Construct moveit_simple_controller_manager dict
    controllers_config = {}
    if "controller_names" in moveit_params:
        controllers_config["controller_names"] = moveit_params["controller_names"]
    if "joint_trajectory_controller" in moveit_params:
        controllers_config["joint_trajectory_controller"] = moveit_params["joint_trajectory_controller"]
    if "gripper_position_controller" in moveit_params:
        controllers_config["gripper_position_controller"] = moveit_params["gripper_position_controller"]

    moveit_params.update({
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
        "robot_description_kinematics": {
            "ur5_manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
                "kinematics_solver_attempts": 10,
            }
        }
    })

    pick_place_node = Node(
        package="ur_yt_sim",
        executable="pick_place",
        name="pick_and_place_node",
        output="screen",
        parameters=[moveit_params],
    )

    return LaunchDescription([pick_place_node])
