// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <cartesian_mover/action/execute_task.hpp>
// #include "ament_index_cpp/get_package_share_directory.hpp"
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <yaml-cpp/yaml.h>
// #include <filesystem>
// #include <fstream>
// #include <thread>
// #include <map>

// namespace fs = std::filesystem;

// class TaskExecutorServer : public rclcpp::Node {
// public:
//   using ExecuteTask = cartesian_mover::action::ExecuteTask;
//   using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

//   explicit TaskExecutorServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
//       : Node("task_executor_server", options),
//         move_group_node_(std::make_shared<rclcpp::Node>("move_group_node")),
//         executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {

//     executor_->add_node(move_group_node_);
//     executor_thread_ = std::thread([this]() { executor_->spin(); });

//     // Initialize MoveGroupInterfaces
//     planning_groups_ = {
//         {"arm", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "arm")},
//         {"arm_torso", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "arm_torso")},
//         {"gripper", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "gripper")}
//     };

//     // Create action server
//     action_server_ = rclcpp_action::create_server<ExecuteTask>(
//         this,
//         "execute_task",
//         std::bind(&TaskExecutorServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
//         std::bind(&TaskExecutorServer::handle_cancel, this, std::placeholders::_1),
//         std::bind(&TaskExecutorServer::handle_accepted, this, std::placeholders::_1)
//     );
//   }

//   ~TaskExecutorServer() {
//     executor_->cancel();
//     executor_thread_.join();
//   }

// private:
//   rclcpp::Node::SharedPtr move_group_node_;
//   std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
//   std::thread executor_thread_;
//   std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> planning_groups_;
//   rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

//   rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ExecuteTask::Goal> goal) {
//     RCLCPP_INFO(this->get_logger(), "Received goal request: %s", goal->go_to.c_str());
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
//     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }

//   void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
//     std::thread{std::bind(&TaskExecutorServer::execute, this, std::placeholders::_1), goal_handle}.detach();
//   }

//   bool load_and_execute_task(const std::string &task_name,
//                            std::shared_ptr<ExecuteTask::Feedback> feedback,
//                            std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
//     std::string path = ament_index_cpp::get_package_share_directory("cartesian_mover") + "/config/tiago_tasks.yaml";
//     if (!fs::exists(path)) {
//         RCLCPP_ERROR(this->get_logger(), "YAML file not found: %s", path.c_str());
//         return false;
//     }

//     YAML::Node root = YAML::LoadFile(path);
//     if (!root[task_name]) {
//         RCLCPP_ERROR(this->get_logger(), "Task '%s' not found in YAML file.", task_name.c_str());
//         return false;
//     }

//     YAML::Node actions = root[task_name];
//     for (const auto &action : actions) {
//         std::string action_name = action.first.as<std::string>();
//         std::string group = action.second["group"].as<std::string>();
//         std::vector<double> joints = action.second["joints"].as<std::vector<double>>();

//         feedback->feedback = "Executing: " + action_name;
//         goal_handle->publish_feedback(feedback);

//         RCLCPP_INFO(this->get_logger(), "  - Subtask: %s (Group: %s)", action_name.c_str(), group.c_str());
//         if (!moveToJoints(group, joints)) {
//         return false;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     }

//     return true;
//     }

//   void execute(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
//     RCLCPP_INFO(this->get_logger(), "Executing task");

//     const auto goal = goal_handle->get_goal();
//     auto feedback = std::make_shared<ExecuteTask::Feedback>();
//     auto result = std::make_shared<ExecuteTask::Result>();

//     // Load and execute the specified task
//     bool success = load_and_execute_task(goal->go_to, feedback, goal_handle);

//     if (success) {
//       result->success = true;
//       result->message = "Task completed successfully.";
//       RCLCPP_INFO(this->get_logger(), "Task succeeded");
//       goal_handle->succeed(result);
//     } else {
//       result->success = false;
//       result->message = "Task execution failed.";
//       RCLCPP_ERROR(this->get_logger(), "Task failed");
//       goal_handle->abort(result);
//     }
//   }

//   bool moveToJoints(const std::string &group_name, const std::vector<double> &joint_values) {
//     auto it = planning_groups_.find(group_name);
//     if (it == planning_groups_.end()) {
//       RCLCPP_ERROR(this->get_logger(), "Unknown planning group: %s", group_name.c_str());
//       return false;
//     }
  
//     auto &group = *(it->second);
//     group.setJointValueTarget(joint_values);
  
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (success) {
//       RCLCPP_INFO(this->get_logger(), "Executing plan for group: %s", group_name.c_str());
//       group.execute(plan);
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "Planning failed for group: %s", group_name.c_str());
//     }
  
//     return success;
//   }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TaskExecutorServer>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }



#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cartesian_mover/action/execute_task.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <thread>
#include <map>

namespace fs = std::filesystem;

class TaskExecutorServer : public rclcpp::Node {
public:
  using ExecuteTask = cartesian_mover::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit TaskExecutorServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("task_executor_server", options),
        move_group_node_(std::make_shared<rclcpp::Node>("move_group_node")),
        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
        planning_scene_interface_() {

    executor_->add_node(move_group_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    planning_groups_ = {
        {"arm", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "arm")},
        {"arm_torso", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "arm_torso")},
        {"gripper", std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "gripper")}
    };

    action_server_ = rclcpp_action::create_server<ExecuteTask>(
        this,
        "execute_task",
        std::bind(&TaskExecutorServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskExecutorServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&TaskExecutorServer::handle_accepted, this, std::placeholders::_1)
    );

    add_collision_objects();
  }

  ~TaskExecutorServer() {
    executor_->cancel();
    executor_thread_.join();
  }

private:
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> planning_groups_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

  void add_collision_objects() {
    moveit_msgs::msg::CollisionObject table, cylinder;
    table.id = "table";
    table.header.frame_id = "base_footprint";

    shape_msgs::msg::SolidPrimitive table_shape;
    table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_shape.dimensions = {0.8, 0.6, 0.05};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.6;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.4;
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(table_shape);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    cylinder.id = "cylinder";
    cylinder.header.frame_id = "base_footprint";

    shape_msgs::msg::SolidPrimitive cylinder_shape;
    cylinder_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder_shape.dimensions = {0.2, 0.03};

    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.position.x = 0.6;
    cylinder_pose.position.y = 0.0;
    cylinder_pose.position.z = 0.55;
    cylinder_pose.orientation.w = 1.0;

    cylinder.primitives.push_back(cylinder_shape);
    cylinder.primitive_poses.push_back(cylinder_pose);
    cylinder.operation = cylinder.ADD;

    planning_scene_interface_.applyCollisionObjects({table, cylinder});
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request: %s", goal->go_to.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask>) {
    RCLCPP_INFO(this->get_logger(), "Cancel goal request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  bool load_and_execute_task(const std::string &task_name,
                             std::shared_ptr<ExecuteTask::Feedback> feedback,
                             std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
    std::string path = ament_index_cpp::get_package_share_directory("cartesian_mover") + "/config/pick_place_tasks.yaml";
    if (!fs::exists(path)) {
        RCLCPP_ERROR(this->get_logger(), "YAML file not found: %s", path.c_str());
        return false;
    }

    YAML::Node root = YAML::LoadFile(path);
    if (!root[task_name]) {
        RCLCPP_ERROR(this->get_logger(), "Task '%s' not found", task_name.c_str());
        return false;
    }

    YAML::Node actions = root[task_name];
    for (const auto &action : actions) {
        std::string action_name = action.first.as<std::string>();
        std::string group = action.second["group"].as<std::string>();
        std::vector<double> joints = action.second["joints"].as<std::vector<double>>();


        if (action_name == "close_gripper") {
          // ATTACH CYLINDER TO GRIPPER
          moveit_msgs::msg::AttachedCollisionObject attached_object;
          attached_object.link_name = "gripper_grasping_frame";  // replace with actual gripper tip link
          attached_object.object.id = "cylinder";
          attached_object.object.header.frame_id = "gripper_grasping_frame";
          attached_object.object.operation = attached_object.object.ADD;
      
          shape_msgs::msg::SolidPrimitive primitive;
          primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
          primitive.dimensions = {0.2, 0.03};
      
          geometry_msgs::msg::Pose relative_pose;
          relative_pose.orientation.w = 1.0;
          relative_pose.position.z = 0.1;
      
          attached_object.object.primitives.push_back(primitive);
          attached_object.object.primitive_poses.push_back(relative_pose);
      
          planning_scene_interface_.applyAttachedCollisionObject(attached_object);
        }   
        // if (action_name == "open_gripper") {
        //   moveit_msgs::msg::AttachedCollisionObject detach_object;
        //   detach_object.object.id = "cylinder";
        //   detach_object.link_name = "gripper_grasping_frame";  // Use your actual link name here
        //   detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        
        //   planning_scene_interface_.applyAttachedCollisionObject(detach_object);
        // }

        if (action_name == "open_gripper") {
          // Create an AttachedCollisionObject message to detach the object
          moveit_msgs::msg::AttachedCollisionObject detach_object;
          detach_object.object.id = "cylinder";  // ID of the object to detach
          detach_object.link_name = "gripper_grasping_frame";  // Link to which the object is attached
          detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;  // Operation to remove the object
        
          // Apply the attached collision object to detach it
          planning_scene_interface_.applyAttachedCollisionObject(detach_object);
        
          // Optionally, remove the object from the planning scene entirely
          planning_scene_interface_.removeCollisionObjects({"cylinder"});
        }
        

        feedback->feedback = "Executing: " + action_name;
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Subtask: %s -> Group: %s", action_name.c_str(), group.c_str());
        if (!moveToJoints(group, joints)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return true;
  }

  void execute(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    auto result = std::make_shared<ExecuteTask::Result>();

    bool success = load_and_execute_task(goal->go_to, feedback, goal_handle);

    if (success) {
        result->success = true;
        result->message = "Task completed successfully.";
        goal_handle->succeed(result);
    } else {
        result->success = false;
        result->message = "Task execution failed.";
        goal_handle->abort(result);
    }
  }

  bool moveToJoints(const std::string &group_name, const std::vector<double> &joint_values) {
    auto it = planning_groups_.find(group_name);
    if (it == planning_groups_.end()) return false;

    auto &group = *(it->second);
    group.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        group.execute(plan);
        return true;
    }
    return false;
  }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskExecutorServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

