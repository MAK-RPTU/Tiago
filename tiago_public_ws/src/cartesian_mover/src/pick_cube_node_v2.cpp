#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thread>
#include <vector>
#include <string>

class PickCubeWithMoveIt : public rclcpp::Node {
public:
    PickCubeWithMoveIt(const rclcpp::NodeOptions &options)
        : Node("pick_cube_node", options),
          move_group_node_(std::make_shared<rclcpp::Node>("move_group_node", rclcpp::NodeOptions().parameter_overrides({
          {"use_sim_time", rclcpp::ParameterValue(true)}
          }))),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
          arm_group(move_group_node_, "arm"),
          gripper_group(move_group_node_, "gripper"),
          arm_torso_group(move_group_node_, "arm_torso") {

        arm_group.setEndEffectorLink("gripper_grasping_frame");
        client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");

        executor_->add_node(move_group_node_);
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });

        wait_for_valid_robot_state();
        wait_for_service();
    }

    ~PickCubeWithMoveIt() {
        executor_->cancel();
        executor_thread_.join();
    }

    void pickAndPlaceCube() {
        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = "aruco_cube";
        request->reference_frame = "base_footprint";

        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call /gazebo/get_entity_state");
            return;
        }

        auto response = future.get();
        auto cube_pose = response->state.pose;

        std::vector<double> pre_grasp_joints = {0.34, 0.45, -0.42, -2.56, 1.99, 0.64, -1.04, -0.44};
        arm_torso_group.setJointValueTarget(pre_grasp_joints);
        if (arm_torso_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to pre-grasp joint configuration.");
            return;
        }

        // gripper_group.setJointValueTarget({0.04, 0.04});
        std::vector<double> open_grip = {0.04, 0.04};
        gripper_group.setJointValueTarget(open_grip);
        gripper_group.move();

        geometry_msgs::msg::PoseStamped grasp_pose;
        grasp_pose.header.frame_id = "base_footprint";
        grasp_pose.pose = cube_pose;
        grasp_pose.pose.position.z += 0.015;
        grasp_pose.pose.orientation.w = 1.0;
        arm_group.setPoseTarget(grasp_pose);
        arm_group.move();

        // gripper_group.setJointValueTarget({0.0, 0.0});
        std::vector<double> close_grip = {0.0, 0.0};
        gripper_group.setJointValueTarget(close_grip);
        gripper_group.move();

        geometry_msgs::msg::PoseStamped lift_pose = grasp_pose;
        lift_pose.pose.position.z += 0.15;
        arm_group.setPoseTarget(lift_pose);
        arm_group.move();

        geometry_msgs::msg::PoseStamped place_pose;
        place_pose.header.frame_id = "base_footprint";
        place_pose.pose.position.x = 0.5;
        place_pose.pose.position.y = -0.3;
        place_pose.pose.position.z = 0.85;
        place_pose.pose.orientation.w = 1.0;
        arm_group.setPoseTarget(place_pose);
        arm_group.move();

        // gripper_group.setJointValueTarget({0.04, 0.04});
        // std::vector<double> open_grip = {0.04, 0.04};
        gripper_group.setJointValueTarget(open_grip);
        gripper_group.move();

        RCLCPP_INFO(this->get_logger(), "✅ Pick-and-place complete.");
    }

private:
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_;
    rclcpp::Node::SharedPtr move_group_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;
    moveit::planning_interface::MoveGroupInterface arm_torso_group;

    void wait_for_service() {
        while (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /gazebo/get_entity_state service...");
        }
    }

    void wait_for_valid_robot_state() {
        RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt to receive a valid robot state...");
        while (rclcpp::ok()) {
            auto current_state = arm_group.getCurrentState(10);
            if (current_state) {
                RCLCPP_INFO(this->get_logger(), "Successfully received robot state!");
                return;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<PickCubeWithMoveIt>(node_options);
    node->pickAndPlaceCube();
    rclcpp::shutdown();
    return 0;
}