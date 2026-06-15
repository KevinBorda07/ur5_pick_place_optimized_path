#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <chrono>
#include <thread>
#include <future>

using namespace std::chrono_literals;

class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
        : move_group(node, "ur5_manipulator"),
          gripper(node, "robotiq_gripper"),
          planning_scene_interface(),
          logger(rclcpp::get_logger("PickAndPlace")),
          node_(node)
    {
        move_group.setPoseReferenceFrame("world");
        
        // Create service clients
        attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
        spawn_client = node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        delete_client = node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
        set_entity_state_client = node_->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

        // Create gripper action client
        gripper_action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
            node_, "/gripper_position_controller/gripper_cmd");
    }

    template <typename ClientType, typename RequestType>
    bool call_service(ClientType client, RequestType request, const std::string& service_name)
    {
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(logger, "Interrupted while waiting for service %s.", service_name.c_str());
                return false;
            }
            RCLCPP_WARN(logger, "Waiting for service %s...", service_name.c_str());
        }
        auto future = client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
        {
            return future.get()->success;
        }
        RCLCPP_ERROR(logger, "Service call to %s timed out.", service_name.c_str());
        return false;
    }

    void send_gripper_goal(double position, double max_effort = 20.0, bool wait_for_result = true)
    {
        if (!gripper_action_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(logger, "Gripper action server not available!");
            return;
        }
        auto goal_msg = control_msgs::action::GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = max_effort;

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
        auto future_goal_handle = gripper_action_client->async_send_goal(goal_msg, send_goal_options);
        
        // Wait for goal response
        if (future_goal_handle.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(logger, "Timeout waiting for gripper goal response");
            return;
        }

        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(logger, "Gripper goal rejected by server");
            return;
        }

        if (!wait_for_result) {
            return;
        }

        // Wait for execution result
        auto future_result = gripper_action_client->async_get_result(goal_handle);
        if (future_result.wait_for(std::chrono::seconds(20)) != std::future_status::ready) {
            RCLCPP_ERROR(logger, "Timeout waiting for gripper execution result");
            return;
        }
    }

    void close_gripper()
    {
        RCLCPP_INFO(logger, "Closing gripper...");
        // Pre-grasp close
        send_gripper_goal(0.45, 10.0, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        // Force contact
        send_gripper_goal(0.48, 10.0, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void open_gripper()
    {
        RCLCPP_INFO(logger, "Opening gripper...");
        send_gripper_goal(0.0, 20.0, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void move_to_home()
    {
        RCLCPP_INFO(logger, "Moving to Home position...");
        move_group.setNamedTarget("home");
        move_group.move();
    }

    bool pick()
    {
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
        move_group.setPlanningTime(10.0);
        move_group.setNumPlanningAttempts(10);
        move_group.allowReplanning(true);
        move_group.setGoalTolerance(0.01);

        tf2::Quaternion orientation;
        orientation.setRPY(-3.1415, 0, 0.0);

        geometry_msgs::msg::Pose hover_pose;
        hover_pose.orientation = tf2::toMsg(orientation);
        hover_pose.position.x = 0.50;
        hover_pose.position.y = 0.0;
        hover_pose.position.z = 1.50;

        geometry_msgs::msg::Pose pick_pose;
        pick_pose.orientation = tf2::toMsg(orientation);
        pick_pose.position.x = 0.50;
        pick_pose.position.y = 0.0;
        pick_pose.position.z = 1.25;

        // Plan joint trajectory to pick pose
        move_group.setPoseTarget(pick_pose, "wrist_3_link");
        moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
        bool plan_ok = (move_group.plan(pick_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!plan_ok) {
            RCLCPP_ERROR(logger, "Initial plan to pick pose failed!");
            return false;
        }

        // Get joint values at pick pose
        std::vector<double> q_pick = pick_plan.trajectory_.joint_trajectory.points.back().positions;

        // Set start state to target
        moveit::core::RobotState temp_state(*move_group.getCurrentState());
        temp_state.setJointGroupPositions("ur5_manipulator", q_pick);
        move_group.setStartState(temp_state);

        // Plan linear path to hover pose
        std::vector<geometry_msgs::msg::Pose> waypoints_up;
        waypoints_up.push_back(hover_pose);
        moveit_msgs::msg::RobotTrajectory traj_up;
        double fraction_up = move_group.computeCartesianPath(
            waypoints_up, 0.005, 4.0, traj_up);

        RCLCPP_INFO(logger, "Upward Cartesian path coverage: %.1f%%", fraction_up * 100.0);

        std::vector<double> q_hover;
        moveit_msgs::msg::RobotTrajectory traj_down;
        bool cartesian_success = false;

        if (fraction_up > 0.99) {
            // Get joint values at hover pose
            q_hover = traj_up.joint_trajectory.points.back().positions;

            // Plan linear path to pick pose
            temp_state.setJointGroupPositions("ur5_manipulator", q_hover);
            move_group.setStartState(temp_state);

            std::vector<geometry_msgs::msg::Pose> waypoints_down;
            waypoints_down.push_back(pick_pose);
            double fraction_down = move_group.computeCartesianPath(
                waypoints_down, 0.005, 4.0, traj_down);

            RCLCPP_INFO(logger, "Downward Cartesian path coverage: %.1f%%", fraction_down * 100.0);
            if (fraction_down > 0.99) {
                cartesian_success = true;
            }
        }

        // Reset start state
        move_group.setStartStateToCurrentState();

        if (cartesian_success) {
            RCLCPP_INFO(logger, "Cartesian descent planning SUCCESS. Moving to q_hover...");
            move_group.setJointValueTarget(q_hover);
            moveit::planning_interface::MoveGroupInterface::Plan plan_to_hover;
            if (move_group.plan(plan_to_hover) == moveit::core::MoveItErrorCode::SUCCESS) {
                move_group.execute(plan_to_hover);
            } else {
                RCLCPP_ERROR(logger, "Failed to plan to q_hover!");
                return false;
            }

            // Open gripper
            send_gripper_goal(0.0, 10.0, true);

            // Remove collision object
            std::vector<std::string> obj_to_remove = {"cube_pick"};
            planning_scene_interface.removeCollisionObjects(obj_to_remove);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // Execute descent
            RCLCPP_INFO(logger, "Executing Cartesian descent straight down...");
            moveit::planning_interface::MoveGroupInterface::Plan descent_plan;
            descent_plan.trajectory_ = traj_down;
            move_group.execute(descent_plan);
        } else {
            RCLCPP_WARN(logger, "Cartesian descent planning failed, falling back to default pose planning");
            move_group.setStartStateToCurrentState();
            
            // Fallback to joint planning
            move_group.setPoseTarget(hover_pose, "wrist_3_link");
            move_group.move();

            send_gripper_goal(0.0, 10.0, true);

            std::vector<std::string> obj_to_remove = {"cube_pick"};
            planning_scene_interface.removeCollisionObjects(obj_to_remove);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            move_group.setPoseTarget(pick_pose, "wrist_3_link");
            move_group.move();
        }
        RCLCPP_INFO(logger, "Pick motion execution completed.");

        // Grasping sequence
        RCLCPP_INFO(logger, "Closing gripper to pre-grasp position (0.46)...");
        send_gripper_goal(0.46, 5.0, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        RCLCPP_INFO(logger, "Attaching object...");
        attachObject();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        RCLCPP_INFO(logger, "Closing gripper gently to contact position (0.52)...");
        send_gripper_goal(0.52, 2.0, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));

        // Attach collision object
        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name = "wrist_3_link";
        aco.object.id = "cube_pick";
        aco.object.header.frame_id = "tool0";
        aco.object.primitives.resize(1);
        aco.object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        aco.object.primitives[0].dimensions = {0.04, 0.04, 0.04};
        aco.object.primitive_poses.resize(1);
        aco.object.primitive_poses[0].position.x = 0.0;
        aco.object.primitive_poses[0].position.y = 0.0;
        aco.object.primitive_poses[0].position.z = 0.20;
        aco.object.primitive_poses[0].orientation.w = 1.0;
        aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        aco.touch_links = {"wrist_3_link", "left_inner_finger", "left_inner_finger_pad",
                           "right_inner_finger", "right_inner_finger_pad",
                           "left_outer_finger", "right_outer_finger",
                           "left_inner_knuckle", "right_inner_knuckle",
                           "left_outer_knuckle", "right_outer_knuckle",
                           "robotiq_arg2f_base_link", "table1", "tool0"};
        planning_scene_interface.applyAttachedCollisionObject(aco);
        RCLCPP_INFO(logger, "Cube attached to MoveIt planning scene.");
        std::this_thread::sleep_for(0.5s);

        // Lift arm
        RCLCPP_INFO(logger, "Lifting after pick...");
        geometry_msgs::msg::Pose lift_pose;
        lift_pose.orientation = tf2::toMsg(orientation);
        lift_pose.position.x = 0.50;
        lift_pose.position.y = 0.0;
        lift_pose.position.z = 1.50;
        move_group.setPoseTarget(lift_pose, "wrist_3_link");
        auto lift_result = move_group.move();
        if (lift_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(logger, "Lift motion failed, continuing anyway...");
        }

        return true;
    }

    // Allow or disallow gripper-table collision
    void allowGripperTableCollision(bool allow)
    {
        moveit_msgs::msg::PlanningScene scene_diff;
        scene_diff.is_diff = true;

        const std::vector<std::string> gripper_links = {
            "left_inner_finger_pad", "right_inner_finger_pad",
            "left_inner_finger", "right_inner_finger",
            "left_outer_finger", "right_outer_finger",
            "left_inner_knuckle", "right_inner_knuckle",
            "left_outer_knuckle", "right_outer_knuckle",
            "robotiq_arg2f_base_link", "wrist_3_link"
        };

        moveit_msgs::msg::AllowedCollisionMatrix acm;
        acm.entry_names.push_back("table1");
        for (const auto & link : gripper_links) {
            acm.entry_names.push_back(link);
        }

        // Build allowed collision matrix
        size_t n = acm.entry_names.size();
        for (size_t i = 0; i < n; ++i) {
            moveit_msgs::msg::AllowedCollisionEntry row;
            row.enabled.resize(n, false);
            acm.entry_values.push_back(row);
        }
        
        // Set allowed entries
        for (size_t j = 1; j < n; ++j) {
            acm.entry_values[0].enabled[j] = allow;
            acm.entry_values[j].enabled[0] = allow;
        }

        scene_diff.allowed_collision_matrix = acm;

        auto ps_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 1);
        std::this_thread::sleep_for(100ms);
        ps_pub->publish(scene_diff);
        RCLCPP_INFO(logger, "ACM updated: gripper-table collision %s", allow ? "ALLOWED" : "RESTORED");
    }

    bool place()
    {
        std::this_thread::sleep_for(1s);
        move_group.setMaxVelocityScalingFactor(0.8);
        move_group.setMaxAccelerationScalingFactor(0.8);
        move_group.setPlanningTime(10.0);  
        move_group.setNumPlanningAttempts(10);
        move_group.allowReplanning(true);  
        move_group.setGoalTolerance(0.01); 

        geometry_msgs::msg::Pose place_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-3.1415, 0, 0.0);
        place_pose.orientation = tf2::toMsg(orientation);
        place_pose.position.x = 0.60;
        place_pose.position.y = 0.40;
        place_pose.position.z = 1.28;

        move_group.setPoseTarget(place_pose, "wrist_3_link");

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing place plan: %s", success ? "SUCCESS" : "FAILED");

        // Execute
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Motion execution completed.");
            
            detachObject();
            std::this_thread::sleep_for(1s);

            open_gripper();
            std::this_thread::sleep_for(1s);
            return true;
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning failed!");
            return false;
        }
    }

    void attachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "cobot";
        request->link1_name = "wrist_3_link";
        request->model2_name = "cube_pick";
        request->link2_name = "link_1";

        if (call_service(attach_client, request, "/ATTACHLINK"))
        {
            RCLCPP_INFO(logger, "Object attached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to attach object.");
        }
    }

    void detachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "cobot";
        request->link1_name = "wrist_3_link";
        request->model2_name = "cube_pick";
        request->link2_name = "link_1";

        if (call_service(detach_client, request, "/DETACHLINK"))
        {
            RCLCPP_INFO(logger, "Object detached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to detach object.");
        }

        // Detach from MoveIt
        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name = "wrist_3_link";
        aco.object.id = "cube_pick";
        aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(aco);
        RCLCPP_INFO(logger, "Cube detached from MoveIt planning scene.");
    }

    void addDefaultCollisionObjects()
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects(2);

        // Table
        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "world";
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[0].primitives[0].dimensions = {0.8, 1.5, 1.015};
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.7168;
        collision_objects[0].primitive_poses[0].position.y = -0.0053;
        collision_objects[0].primitive_poses[0].position.z = 0.5075;
        collision_objects[0].primitive_poses[0].orientation.w = 1.0;
        collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

        // Target cube
        collision_objects[1].id = "cube_pick";
        collision_objects[1].header.frame_id = "world";
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[1].primitives[0].dimensions = {0.04, 0.04, 0.04};
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = 0.5;
        collision_objects[1].primitive_poses[0].position.y = 0.0;
        collision_objects[1].primitive_poses[0].position.z = 1.035;
        collision_objects[1].primitive_poses[0].orientation.w = 1.0;
        collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

        planning_scene_interface.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(logger, "Default collision objects added to the planning scene.");
    }

    void spawnObstacle()
    {
        RCLCPP_INFO(logger, "Spawning obstacle in Gazebo...");
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "obstacle_box";
        request->xml = R"(
<sdf version="1.6">
  <model name="obstacle_box">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.1 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.1 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
)";
        // Obstacle position
        request->initial_pose.position.x = 0.55;
        request->initial_pose.position.y = 0.20;
        request->initial_pose.position.z = 1.165;
        request->initial_pose.orientation.w = 1.0;

        if (call_service(spawn_client, request, "/spawn_entity")) {
            RCLCPP_INFO(logger, "Obstacle spawned in Gazebo.");
        } else {
            RCLCPP_ERROR(logger, "Failed to spawn obstacle.");
        }

        // MoveIt planning scene
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";
        collision_object.id = "obstacle_box";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.2, 0.1, 0.3};

        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = 0.55;
        box_pose.position.y = 0.20;
        box_pose.position.z = 1.165;
        box_pose.orientation.w = 1.0;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::msg::CollisionObject> objs;
        objs.push_back(collision_object);
        planning_scene_interface.applyCollisionObjects(objs);
        RCLCPP_INFO(logger, "Obstacle added to MoveIt planning scene.");
    }

    void deleteObstacle()
    {
        RCLCPP_INFO(logger, "Deleting obstacle from Gazebo...");
        auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = "obstacle_box";

        call_service(delete_client, request, "/delete_entity");

        // Remove from MoveIt
        std::vector<std::string> ids = {"obstacle_box"};
        planning_scene_interface.removeCollisionObjects(ids);
        RCLCPP_INFO(logger, "Obstacle removed from MoveIt planning scene.");
    }

    void resetCubePose()
    {
        RCLCPP_INFO(logger, "Resetting green cube pose for next phase...");

        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = "cube_pick";
        request->state.pose.position.x = 0.5;
        request->state.pose.position.y = 0.0;
        request->state.pose.position.z = 1.035;
        request->state.pose.orientation.x = 0.0;
        request->state.pose.orientation.y = 0.0;
        request->state.pose.orientation.z = 0.0;
        request->state.pose.orientation.w = 1.0;
        request->state.twist.linear.x = 0.0;
        request->state.twist.linear.y = 0.0;
        request->state.twist.linear.z = 0.0;
        request->state.twist.angular.x = 0.0;
        request->state.twist.angular.y = 0.0;
        request->state.twist.angular.z = 0.0;
        request->state.reference_frame = "world";

        if (call_service(set_entity_state_client, request, "/gazebo/set_entity_state"))
        {
            RCLCPP_INFO(logger, "Green cube pose teleported successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to teleport green cube pose.");
        }

        // Update planning scene
        moveit_msgs::msg::CollisionObject cube_obj;
        cube_obj.id = "cube_pick";
        cube_obj.header.frame_id = "world";
        cube_obj.primitives.resize(1);
        cube_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        cube_obj.primitives[0].dimensions = {0.04, 0.04, 0.04};
        cube_obj.primitive_poses.resize(1);
        cube_obj.primitive_poses[0].position.x = 0.5;
        cube_obj.primitive_poses[0].position.y = 0.0;
        cube_obj.primitive_poses[0].position.z = 1.035;
        cube_obj.primitive_poses[0].orientation.w = 1.0;
        cube_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        planning_scene_interface.applyCollisionObjects({cube_obj});

        std::this_thread::sleep_for(1s);
        RCLCPP_INFO(logger, "Green cube reset completed.");
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    rclcpp::Logger logger;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_client;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Create node with parameter overrides
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("pick_and_place_node", node_options);

    // Spin node in background
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    PickAndPlace pap(node);

    // Initial setup
    pap.detachObject();
    pap.open_gripper();
    pap.deleteObstacle();
    pap.move_to_home();
    pap.addDefaultCollisionObjects();
    
    std::this_thread::sleep_for(1s);

    // Reset environment
    pap.resetCubePose();

    // Phase 1: Without Obstacle
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Phase 1: Without Obstacle");

    if (pap.pick()) {
        pap.place();
    }
    pap.move_to_home();

    RCLCPP_INFO(rclcpp::get_logger("main"), "Phase 1 Completed.");
    std::this_thread::sleep_for(5s);

    // Reset environment
    pap.detachObject();
    pap.open_gripper();
    pap.resetCubePose();
    std::this_thread::sleep_for(2s);

    // Phase 2: With Obstacle
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Phase 2: With Obstacle");

    pap.spawnObstacle();
    std::this_thread::sleep_for(2s);

    pap.move_to_home();
    if (pap.pick()) {
        pap.place();
    }
    pap.move_to_home();

    // Clean up
    pap.deleteObstacle();
    std::this_thread::sleep_for(1s);

    RCLCPP_INFO(rclcpp::get_logger("main"), "All phases completed successfully.");

    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    return 0;
}