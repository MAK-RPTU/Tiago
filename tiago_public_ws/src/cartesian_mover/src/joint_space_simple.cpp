#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

static const std::string PLANNING_GROUP_ARM = "arm";

class TestTrajectory : public rclcpp::Node {
public:
    TestTrajectory(const rclcpp::NodeOptions &options)
        : Node("joint_space_simple", options),
          move_group_node_(std::make_shared<rclcpp::Node>("move_group_node")),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
          move_group_arm(move_group_node_, PLANNING_GROUP_ARM) {

        RCLCPP_INFO(get_logger(), "Initializing MoveIt...");

        // Start a separate thread for MoveIt node
        executor_->add_node(move_group_node_);
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });

        // Wait until MoveIt receives valid joint states
        wait_for_valid_robot_state();

        // Retrieve joint model group
        auto current_state = move_group_arm.getCurrentState(10);
        if (current_state) {
            joint_model_group_arm = current_state->getJointModelGroup(PLANNING_GROUP_ARM);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get current robot state.");
        }

        // Timer to periodically update state
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TestTrajectory::timer_callback, this));

        get_info();

        // Move to the waypoints
        move_to_waypoints();
    }

    ~TestTrajectory() {
        RCLCPP_INFO(get_logger(), "Shutting down MoveIt node.");
        executor_->cancel();
        executor_thread_.join();
    }

    // Retrieve basic robot information
    void get_info() {
        RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "End-effector link: %s", move_group_arm.getEndEffectorLink().c_str());

        RCLCPP_INFO(get_logger(), "Available Planning Groups:");
        for (const auto &group_name : move_group_arm.getJointModelGroupNames()) {
            RCLCPP_INFO(get_logger(), " - %s", group_name.c_str());
        }
    }

    // Move the robot through predefined waypoints
    void move_to_waypoints() {
        std::vector<std::vector<double>> waypoints = {
            {90 * (M_PI / 180.0), -15 * (M_PI / 180.0), 0, 40 * (M_PI / 180.0), 0, 0, 0}, // First waypoint
            {0, 0, 0, 0, 0, 0, 0}  // Home position
        };

        for (size_t i = 0; i < waypoints.size(); ++i) {
            RCLCPP_INFO(get_logger(), "Moving to Waypoint %lu", i + 1);
            move_group_arm.setJointValueTarget(waypoints[i]);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success) {
                RCLCPP_INFO(get_logger(), "Executing planned trajectory to waypoint %lu...", i + 1);
                move_group_arm.execute(plan);
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to plan trajectory for waypoint %lu.", i + 1);
            }
        }
    }

    // Get the robot's current state
    void current_state() {
        auto current_state_arm = move_group_arm.getCurrentState(10);
        if (!current_state_arm) {
            RCLCPP_ERROR(get_logger(), "Failed to get current state.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Fetching robot's current joint positions.");
        std::vector<double> joint_group_positions;
        current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions);

        for (size_t i = 0; i < joint_group_positions.size(); ++i) {
            RCLCPP_INFO(get_logger(), "Joint[%lu]: %f", i, joint_group_positions[i]);
        }
    }

private:
    rclcpp::Node::SharedPtr move_group_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;

    moveit::planning_interface::MoveGroupInterface move_group_arm;
    const moveit::core::JointModelGroup *joint_model_group_arm{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback() {
        current_state();
    }

    // Function to wait for MoveIt to receive valid joint states
    void wait_for_valid_robot_state() {
        RCLCPP_INFO(get_logger(), "Waiting for MoveIt to receive a valid robot state...");

        while (rclcpp::ok()) {
            auto current_state = move_group_arm.getCurrentState(10);
            if (current_state) {
                RCLCPP_INFO(get_logger(), "Successfully received robot state!");
                return;
            }
            RCLCPP_WARN(get_logger(), "Still waiting for valid joint states...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = std::make_shared<TestTrajectory>(node_options);
    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
    return 0;
}
