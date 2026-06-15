#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo_msgs/srv/set_model_configuration.hpp>
#include <chrono>
#include <memory>

class GazeboJointMirror : public rclcpp::Node
{
public:
    GazeboJointMirror() : Node("gazebo_joint_mirror"), request_in_progress_(false)
    {
        // Create service client
        client_ = this->create_client<gazebo_msgs::srv::SetModelConfiguration>("/gazebo/set_model_configuration");

        // Create subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GazeboJointMirror::joint_state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Gazebo Joint Mirror Node initialized.");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (request_in_progress_) {
            return; // Skip this frame to avoid queue buildup
        }

        if (!client_->service_is_ready()) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for /gazebo/set_model_configuration service...");
            return;
        }

        // Find the finger_joint position
        double finger_joint_pos = 0.0;
        bool found = false;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "finger_joint") {
                finger_joint_pos = msg->position[i];
                found = true;
                break;
            }
        }

        if (!found) {
            return;
        }

        static double last_sent_pos = -999.0;
        if (std::abs(finger_joint_pos - last_sent_pos) < 0.001) {
            return;
        }

        last_sent_pos = finger_joint_pos;

        auto request = std::make_shared<gazebo_msgs::srv::SetModelConfiguration::Request>();
        request->model_name = "cobot";
        request->urdf_param_name = "";

        request->joint_names = {
            "right_outer_knuckle_joint",
            "left_inner_knuckle_joint",
            "right_inner_knuckle_joint",
            "left_inner_finger_joint",
            "right_inner_finger_joint"
        };
        request->joint_positions = {
            -finger_joint_pos,
            -finger_joint_pos,
            -finger_joint_pos,
            finger_joint_pos,
            finger_joint_pos
        };

        request_in_progress_ = true;

        // Call asynchronously
        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>::SharedFuture future) {
                request_in_progress_ = false;
                try {
                    auto response = future.get();
                    if (!response->success) {
                        RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to set model configuration in Gazebo: %s", response->status_message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            });
    }

    rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::atomic<bool> request_in_progress_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboJointMirror>());
    rclcpp::shutdown();
    return 0;
}
