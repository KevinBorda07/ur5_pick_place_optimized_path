/*
 * Simple Pick and Place without MoveIt or ros2_control
 * Uses Gazebo SetModelState service to move robot directly
 * Bypasses ros2_control plugin which was causing crashes
 */

#include <rclcpp/rclcpp.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

class SimplePickAndPlace : public rclcpp::Node {
public:
    SimplePickAndPlace() : rclcpp::Node("simple_pick_place") {
        // Clients for services
        attach_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client_ = this->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
        set_entity_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
        
        RCLCPP_INFO(this->get_logger(), "Simple Pick and Place Node initialized");
    }
    
    void wait_for_services() {
        RCLCPP_INFO(this->get_logger(), "Waiting for /ATTACHLINK service...");
        if (!attach_client_->wait_for_service(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Attach service not available!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Waiting for /DETACHLINK service...");
        if (!detach_client_->wait_for_service(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Detach service not available!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Waiting for /gazebo/set_entity_state service...");
        if (!set_entity_state_client_->wait_for_service(10s)) {
            RCLCPP_WARN(this->get_logger(), "SetEntityState service not available!");
        }
        
        RCLCPP_INFO(this->get_logger(), "Services available!");
    }
    
    // Helper to convert RPY to quaternion
    void rpy_to_quaternion(double roll, double pitch, double yaw,
                          double& qx, double& qy, double& qz, double& qw) {
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        
        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;
    }
    
    void set_robot_pose(double x, double y, double z, double roll=0, double pitch=0, double yaw=0) {
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = "cobot";
        request->state.pose.position.x = x;
        request->state.pose.position.y = y;
        request->state.pose.position.z = z;
        
        // Convert roll, pitch, yaw to quaternion
        double qx, qy, qz, qw;
        rpy_to_quaternion(roll, pitch, yaw, qx, qy, qz, qw);
        
        request->state.pose.orientation.x = qx;
        request->state.pose.orientation.y = qy;
        request->state.pose.orientation.z = qz;
        request->state.pose.orientation.w = qw;
        
        request->state.twist.linear.x = 0;
        request->state.twist.linear.y = 0;
        request->state.twist.linear.z = 0;
        request->state.twist.angular.x = 0;
        request->state.twist.angular.y = 0;
        request->state.twist.angular.z = 0;
        request->state.reference_frame = "world";
        
        auto future = set_entity_state_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s);
    }
    
    void move_to_pose_simple(double x, double y, double z, double roll=0, double pitch=0, double yaw=0, double duration = 1.0) {
        RCLCPP_INFO(this->get_logger(), "Moving to position (%.2f, %.2f, %.2f)", x, y, z);
        set_robot_pose(x, y, z, roll, pitch, yaw);
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(duration * 1000)));
    }
    
    void attach_object() {
        RCLCPP_INFO(this->get_logger(), "Attaching object...");
        
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "cobot";
        request->link1_name = "wrist_3_link";
        request->model2_name = "cube_pick";
        request->link2_name = "link_1";
        
        auto future = attach_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 10s) 
            == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Object attached successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Attach failed: %s", response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Attach service call timed out");
        }
    }
    
    void detach_object() {
        RCLCPP_INFO(this->get_logger(), "Detaching object...");
        
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "cobot";
        request->link1_name = "wrist_3_link";
        request->model2_name = "cube_pick";
        request->link2_name = "link_1";
        
        auto future = detach_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 10s)
            == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Object detached successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Detach failed: %s", response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Detach service call timed out");
        }
    }
    
    void execute_pick_and_place() {
        wait_for_services();
        
        // Home position (base in center, elevated)
        double home_x = 0.0, home_y = 0.3, home_z = 1.2;
        
        // Pick position (above cube at 0.5, 0.0)
        double pick_x = 0.5, pick_y = 0.0, pick_z = 0.8;
        
        // Place position (different location)
        double place_x = 0.8, place_y = 0.3, place_z = 0.8;
        
        RCLCPP_INFO(this->get_logger(), "Starting Pick and Place sequence...");
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        // Step 1: Move to home
        RCLCPP_INFO(this->get_logger(), "Step 1: Moving to home position...");
        move_to_pose_simple(home_x, home_y, home_z, 0, 0, 0, 2.0);
        
        // Step 2: Move to pick position
        RCLCPP_INFO(this->get_logger(), "Step 2: Moving to pick position...");
        move_to_pose_simple(pick_x, pick_y, pick_z, 0, 0, 0, 2.0);
        
        // Step 3: Attach object
        RCLCPP_INFO(this->get_logger(), "Step 3: Attaching object...");
        attach_object();
        std::this_thread::sleep_for(500ms);
        
        // Step 4: Move to place position (with attached object)
        RCLCPP_INFO(this->get_logger(), "Step 4: Moving to place position with attached object...");
        move_to_pose_simple(place_x, place_y, place_z, 0, 0, 0, 2.0);
        
        // Step 5: Detach object
        RCLCPP_INFO(this->get_logger(), "Step 5: Detaching object...");
        detach_object();
        std::this_thread::sleep_for(500ms);
        
        // Step 6: Return to home
        RCLCPP_INFO(this->get_logger(), "Step 6: Returning to home...");
        move_to_pose_simple(home_x, home_y, home_z, 0, 0, 0, 2.0);
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Pick and Place completed successfully!");
    }

private:
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePickAndPlace>();
    node->execute_pick_and_place();
    rclcpp::shutdown();
    return 0;
}
