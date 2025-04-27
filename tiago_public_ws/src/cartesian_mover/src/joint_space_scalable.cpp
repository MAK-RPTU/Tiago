#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <thread>
#include <map>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class JointTaskExecutor : public rclcpp::Node {
public:
    JointTaskExecutor(const rclcpp::NodeOptions &options)
        : Node("joint_space_scalable", options),
          move_group_node_(std::make_shared<rclcpp::Node>("move_group_node")),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {

        executor_->add_node(move_group_node_);
        executor_thread_ = std::thread([this]() { executor_->spin(); });

        // Initialize MoveGroupInterfaces
        planning_groups_ = {
            {"arm", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "arm")},
            {"arm_torso", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "arm_torso")},
            {"gripper", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "gripper")}
        };

        // Load and execute task sequence
        load_and_execute_tasks("config/tiago_tasks.yaml");
    }

    ~JointTaskExecutor() {
        executor_->cancel();
        executor_thread_.join();
    }

private:
    rclcpp::Node::SharedPtr move_group_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> planning_groups_;

    bool moveToJoints(const std::string &group_name, const std::vector<double> &joint_values) {
        auto it = planning_groups_.find(group_name);
        if (it == planning_groups_.end()) {
            RCLCPP_ERROR(get_logger(), "Unknown planning group: %s", group_name.c_str());
            return false;
        }

        auto &group = *(it->second);
        group.setJointValueTarget(joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(get_logger(), "Executing plan for group: %s", group_name.c_str());
            group.execute(plan);
        } else {
            RCLCPP_ERROR(get_logger(), "Planning failed for group: %s", group_name.c_str());
        }
        return success;
    }
    
    void load_and_execute_tasks(const std::string &relative_path) {
        std::string full_path = ament_index_cpp::get_package_share_directory("cartesian_mover") + "/" + relative_path;
    
        if (!fs::exists(full_path)) {
            RCLCPP_ERROR(get_logger(), "YAML file not found: %s", full_path.c_str());
            return;
        }
    
        YAML::Node root = YAML::LoadFile(full_path);
    
        for (const auto &task : root) {
            std::string task_name = task.first.as<std::string>();
            YAML::Node actions = task.second;
    
            RCLCPP_INFO(get_logger(), "▶ Executing Task: %s", task_name.c_str());
    
            for (const auto &action : actions) {
                std::string action_name = action.first.as<std::string>();
                std::string group = action.second["group"].as<std::string>();
                std::vector<double> joints = action.second["joints"].as<std::vector<double>>();
    
                RCLCPP_INFO(get_logger(), "  - Subtask: %s (Group: %s)", action_name.c_str(), group.c_str());
                moveToJoints(group, joints);
                std::this_thread::sleep_for(1s);
            }
    
            RCLCPP_INFO(get_logger(), "✅ Finished Task: %s\n", task_name.c_str());
            std::this_thread::sleep_for(2s);  // Delay between tasks
        }
    }
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<JointTaskExecutor>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
