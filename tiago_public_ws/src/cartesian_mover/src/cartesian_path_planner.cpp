// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit_msgs/msg/robot_trajectory.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// class CartesianPathPlanner : public rclcpp::Node
// {
// public:
//     CartesianPathPlanner() : Node("cartesian_path_planner")
//     {
//         // Note: MoveGroupInterface initialization is deferred to a separate method
//         RCLCPP_INFO(this->get_logger(), "Node initialized, deferring MoveGroupInterface setup");
//     }

//     // Initialize MoveGroupInterface after the node is constructed
//     void initializeMoveGroup()
//     {
//         move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");

//         // Set pose reference frame (should match your robot's base frame, e.g., "base_link")
//         // move_group_->setPoseReferenceFrame("base_link");
//         move_group_->setPoseReferenceFrame("gripper_grasping_frame");
//         // Log the planning frame for verification
//         std::string planning_frame = move_group_->getPlanningFrame();
//         RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", planning_frame.c_str());

//         RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully");
//     }

//     bool planCartesianPathRelative(const std::vector<std::vector<double>>& relative_waypoints)
//     {
//         if (!move_group_)
//         {
//             RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
//             return false;
//         }

//         // Get current pose
//         geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
//         RCLCPP_INFO(this->get_logger(),
//                     "Starting Position: x=%.3f, y=%.3f, z=%.3f",
//                     current_pose.pose.position.x,
//                     current_pose.pose.position.y,
//                     current_pose.pose.position.z);

//         // Create list of pose waypoints based on relative offsets
//         std::vector<geometry_msgs::msg::Pose> pose_waypoints;
//         geometry_msgs::msg::Pose current_position = current_pose.pose;
//         for (const auto& rel_waypoint : relative_waypoints)
//         {
//             geometry_msgs::msg::Pose pose;
//             pose.position.x = current_position.position.x + rel_waypoint[0];
//             pose.position.y = current_position.position.y + rel_waypoint[1];
//             pose.position.z = current_position.position.z + rel_waypoint[2];
//             pose.orientation = current_position.orientation; // Maintain current orientation
//             pose_waypoints.push_back(pose);

//             // Update current_position for next iteration
//             current_position = pose;

//             // Log the planned waypoint
//             RCLCPP_INFO(this->get_logger(),
//                         "Waypoint: x=%.3f, y=%.3f, z=%.3f",
//                         pose.position.x, pose.position.y, pose.position.z);
//         }

//         // Compute Cartesian path
//         moveit_msgs::msg::RobotTrajectory trajectory;
//         double fraction = move_group_->computeCartesianPath(
//             pose_waypoints,
//             0.01,  // eef_step (step size in meters)
//             0.0,   // jump_threshold (disable jump)
//             trajectory);

//         if (fraction == 1.0)
//         {
//             RCLCPP_INFO(this->get_logger(), "Full Cartesian path computed successfully (%.2f%%)", fraction * 100.0);
//             plan_ = trajectory;
//             return true;
//         }
//         else
//         {
//             RCLCPP_WARN(this->get_logger(), "Only %.2f%% of the path could be planned", fraction * 100.0);
//             return false;
//         }
//     }

//     void executePlan()
//     {
//         if (!move_group_ || plan_.joint_trajectory.points.empty())
//         {
//             RCLCPP_ERROR(this->get_logger(), "No valid plan or MoveGroupInterface to execute");
//             return;
//         }
//         move_group_->execute(plan_);
//         RCLCPP_INFO(this->get_logger(), "Trajectory execution completed");
//     }

// private:
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; // Corrected to use std::shared_ptr
//     moveit_msgs::msg::RobotTrajectory plan_;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CartesianPathPlanner>();

//     // Initialize MoveGroupInterface after node construction
//     node->initializeMoveGroup();

//     // Define relative waypoints [dx, dy, dz] in meters from current position
//     std::vector<std::vector<double>> relative_waypoints = {
//         // {-0.05, 0.0, 0.0},   // Move 5cm in X
//         {0.0, -0.05, 0.0},   // Then 5cm in Y
//         // {0.0, 0.0, -0.05},  // Then 5cm down in Z
//         // {-0.05, -0.05, 0.0} // Then back 5cm in X and Y
//     };

//     // Plan the path
//     if (node->planCartesianPathRelative(relative_waypoints))
//     {
//         // Execute the plan
//         node->executePlan();
//     }

//     rclcpp::spin(node); // Keep the node alive to process callbacks
//     rclcpp::shutdown();
//     return 0;
// }




// // #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit_msgs/msg/robot_trajectory.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// class CartesianPathPlanner : public rclcpp::Node
// {
// public:
//     CartesianPathPlanner() : Node("cartesian_path_planner")
//     {
//         RCLCPP_INFO(this->get_logger(), "Node initialized, deferring MoveGroupInterface setup");
//     }

//     void initializeMoveGroup()
//     {
//         move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");

//         // Set the end effector link to gripper_grasping_frame
//         move_group_->setEndEffectorLink("gripper_grasping_frame");

//         // Set pose reference frame to base_link
//         move_group_->setPoseReferenceFrame("base_link");

//         // Allow more planning time and replanning
//         move_group_->setPlanningTime(10.0);
//         move_group_->allowReplanning(true);

//         // Log the planning frame and end effector for verification
//         std::string planning_frame = move_group_->getPlanningFrame();
//         std::string end_effector_link = move_group_->getEndEffectorLink();
//         RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", planning_frame.c_str());
//         RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", end_effector_link.c_str());

//         RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully");
//     }

//     bool planCartesianPathRelative()
//     {
//         if (!move_group_)
//         {
//             RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
//             return false;
//         }

//         // Get current pose of the gripper_grasping_frame
//         geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
//         RCLCPP_INFO(this->get_logger(),
//                     "Current Pose Frame: %s", current_pose.header.frame_id.c_str());
//         RCLCPP_INFO(this->get_logger(),
//                     "Starting Position: x=%.3f, y=%.3f, z=%.3f",
//                     current_pose.pose.position.x,
//                     current_pose.pose.position.y,
//                     current_pose.pose.position.z);

//         // Check if the pose is valid (e.g., not all zeros)
//         if (current_pose.pose.position.x == 0.0 && 
//             current_pose.pose.position.y == 0.0 && 
//             current_pose.pose.position.z == 0.0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Invalid starting pose detected (0, 0, 0). Check /joint_states publication.");
//             return false;
//         }

//         // Define the target pose by applying the relative offsets
//         geometry_msgs::msg::Pose target_pose = current_pose.pose;
//         target_pose.position.x += -0.2; // Move -20cm in X
//         target_pose.position.y += -0.2; // Move -20cm in Y
//         target_pose.position.z += 0.1;  // Move +10cm in Z
//         // Orientation remains the same as the current pose

//         // Log the target pose
//         RCLCPP_INFO(this->get_logger(),
//                     "Target Position: x=%.3f, y=%.3f, z=%.3f",
//                     target_pose.position.x,
//                     target_pose.position.y,
//                     target_pose.position.z);

//         // Create a list of waypoints (just the current and target pose for a straight-line path)
//         std::vector<geometry_msgs::msg::Pose> pose_waypoints;
//         pose_waypoints.push_back(current_pose.pose); // Start at current pose
//         pose_waypoints.push_back(target_pose);       // End at target pose

//         // Compute Cartesian path
//         moveit_msgs::msg::RobotTrajectory trajectory;
//         double fraction = move_group_->computeCartesianPath(
//             pose_waypoints,
//             0.01,  // eef_step (step size in meters)
//             0.0,   // jump_threshold (disable jump)
//             trajectory);

//         if (fraction == 1.0)
//         {
//             RCLCPP_INFO(this->get_logger(), "Full Cartesian path computed successfully (%.2f%%)", fraction * 100.0);
//             plan_ = trajectory;
//             return true;
//         }
//         else
//         {
//             RCLCPP_WARN(this->get_logger(), "Only %.2f%% of the path could be planned", fraction * 100.0);
//             return false;
//         }
//     }

//     void executePlan()
//     {
//         if (!move_group_ || plan_.joint_trajectory.points.empty())
//         {
//             RCLCPP_ERROR(this->get_logger(), "No valid plan or MoveGroupInterface to execute");
//             return;
//         }
//         move_group_->execute(plan_);
//         RCLCPP_INFO(this->get_logger(), "Trajectory execution completed");
//     }

// private:
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
//     moveit_msgs::msg::RobotTrajectory plan_;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CartesianPathPlanner>();

//     // Initialize MoveGroupInterface after node construction
//     node->initializeMoveGroup();

//     // Plan the path with the specified relative movement
//     if (node->planCartesianPathRelative())
//     {
//         // Execute the plan
//         node->executePlan();
//     }

//     rclcpp::spin(node); // Keep the node alive to process callbacks
//     rclcpp::shutdown();
//     return 0;
// }



#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cartesian_planning_node");
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    
    // Set the reference frame
    move_group.setPoseReferenceFrame("gripper_grasping_frame");
    
    // Get current end-effector pose
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    
    // Define the target pose (move -0.05m in X direction relative to gripper_grasping_frame)
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x -= 0.05;
    
    // Plan a Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  // No jump threshold
    const double eef_step = 0.01;  // Step size for interpolation
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction > 0.95) // Ensure successful planning
    {
        RCLCPP_INFO(node->get_logger(), "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        move_group.execute(cartesian_plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
    }
    
    rclcpp::shutdown();
    return 0;
}
