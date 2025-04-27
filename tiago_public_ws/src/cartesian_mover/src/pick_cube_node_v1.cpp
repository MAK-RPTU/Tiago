///////////////////This is simple planning using target_pose so its IK//////////////////////

// #include <rclcpp/rclcpp.hpp>
// #include <gazebo_msgs/srv/get_entity_state.hpp>
// #include <geometry_msgs/msg/pose.hpp>

// class LogCubePoseNode : public rclcpp::Node
// {
// public:
//   LogCubePoseNode() : Node("pick_cube_node")
//   {
//     client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");

//     // Wait for the service to be available
//     while (!client_->wait_for_service(std::chrono::seconds(2))) {
//       RCLCPP_WARN(this->get_logger(), "Waiting for /gazebo/get_entity_state service...");
//     }

//     // Create the request
//     auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
//     request->name = "aruco_cube";                    // <- model name
//     request->reference_frame = "base_footprint";     // <- optional frame

//     // Call the service
//     auto future = client_->async_send_request(request);
//     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
//         rclcpp::FutureReturnCode::SUCCESS)
//     {
//       auto response = future.get();
//       const auto &pose = response->state.pose;

//       RCLCPP_INFO(this->get_logger(), "Cube Pose:");
//       RCLCPP_INFO(this->get_logger(), "Position: [x=%.3f, y=%.3f, z=%.3f]",
//                   pose.position.x, pose.position.y, pose.position.z);
//       RCLCPP_INFO(this->get_logger(), "Orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
//                   pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
//     }
//     else
//     {
//       RCLCPP_ERROR(this->get_logger(), "Failed to call /gazebo/get_entity_state");
//     }
//   }

// private:
//   rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<LogCubePoseNode>();
//   rclcpp::spin_some(node);
//   rclcpp::shutdown();
//   return 0;
// }


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

        // Start a separate thread for MoveIt node
        executor_->add_node(move_group_node_);
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });

        // Wait for MoveIt and service
        wait_for_valid_robot_state();
        wait_for_service();

        // Fetch the cube pose and plan a motion
        get_cube_pose_and_move();
    }

    ~PickCubeWithMoveIt() {
        executor_->cancel();
        executor_thread_.join();
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

    // void get_cube_pose_and_move() {
    //     auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    //     request->name = "aruco_cube";
    //     request->reference_frame = "base_footprint";

    //     auto future = client_->async_send_request(request);
    //     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
    //         rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to call /gazebo/get_entity_state");
    //         return;
    //     }
        
    //     // std::vector<double> arm_torso_joint_values = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //     // std::vector<double> arm_torso_joint_values = {0.3, 2.58, 0.37, 1.07, 1.27, -1.69, 1.36, -1.31};
    //     // std::vector<double> arm_torso_joint_values = {0.3, 2.58, 0.68, 1.08, 1.61, -1.95, 1.17, -1.3};
    //     std::vector<double> arm_torso_joint_values = {0.34, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0};
    //     // std::vector<double> arm_torso_joint_values = {0.3, 0.37, 0.54, -2.5, 2.1, -0.49, 1.4, -1.92};
    //     std::vector<std::vector<double>> joint_sequences = {
    //     //   {0.34, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0},
    //     //   {0.34, 0.10, 0.47, -0.20, 1.56, -1.58, 0.25, 0.0},
    //     //   {0.34, 0.10, 0.47, -0.20, 1.56, 1.60, 0.25, 1.19},
    //     //   {0.34, 0.10, 0.47, -0.20, 1.56, 1.60, 0.25, 1.19},
    //     //   {0.34, 0.78, 0.92, 0.28, 1.7, 1.62, -0.10, 1.66},
    //     //   {0.34, 1.47, 0.99, 0.78, 1.78, 1.67, -0.24, 1.92},
    //       {0.34, 0.49, -0.4, -2.56, 1.97, 0.63, -1.03, -0.44}, //Pre_grasp_pose
    //       {0.34, 0.45, -0.42, -2.56, 1.99, 0.64, -1.04, -0.44}, //Pre_grasp_pose
    //     //   {0.34, 0.37, -0.42, -2.56, 2.04, 0.64, -1.03, -0.4}, //Pre_grasp_pose
    //     //   {0.34, 0.37, 0.54, -2.5, 2.1, -0.49, -1.36, -1.92} //Pose above the cube
    //     //   {0.34, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

    //     };
        
    //     for (size_t i = 0; i < joint_sequences.size(); ++i) {
    //         arm_torso_group.setJointValueTarget(joint_sequences[i]);
        
    //         moveit::planning_interface::MoveGroupInterface::Plan plan;
    //         if (arm_torso_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    //             arm_torso_group.move();
    //             RCLCPP_INFO(this->get_logger(), "Moved to joint target %ld.", i);
    //         } else {
    //             RCLCPP_ERROR(this->get_logger(), "Failed to plan to joint target %ld.", i);
    //             break;  // Optional: stop if one fails
    //         }
    //     }

    //     geometry_msgs::msg::PoseStamped current_pose = arm_group.getCurrentPose();
    //     std::string reference_frame = arm_group.getPlanningFrame();
    //     RCLCPP_INFO(this->get_logger(), "Current EE Pose is relative to frame: %s", reference_frame.c_str());
    //     RCLCPP_INFO(this->get_logger(), "Current EE Pose:");
    //     RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
    //                 current_pose.pose.position.x,
    //                 current_pose.pose.position.y,
    //                 current_pose.pose.position.z);
    //     RCLCPP_INFO(this->get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
    //                 current_pose.pose.orientation.x,
    //                 current_pose.pose.orientation.y,
    //                 current_pose.pose.orientation.z,
    //                 current_pose.pose.orientation.w);
        
    //     // Open gripper
    //     std::vector<double> open_grip = {0.04, 0.04};
    //     gripper_group.setJointValueTarget(open_grip);
    //     gripper_group.move();
        
    //     // Relative 10 cm downward in local frame
    //     tf2::Vector3 offset_local(0.0, 0.0, 0.30);
    //     tf2::Quaternion q(
    //         current_pose.pose.orientation.x,
    //         current_pose.pose.orientation.y,
    //         current_pose.pose.orientation.z,
    //         current_pose.pose.orientation.w
    //     );
    //     tf2::Vector3 offset_base = tf2::quatRotate(q, offset_local);
        
    //     geometry_msgs::msg::PoseStamped offset_pose = current_pose;
    //     // offset_pose.pose.position.x += offset_base.x();
    //     // offset_pose.pose.position.y += offset_base.y();
    //     offset_pose.pose.position.z += offset_base.z();
        
    //     RCLCPP_INFO(this->get_logger(), "Offset Pose: x=%.2f, y=%.2f, z=%.2f",
    //         offset_pose.pose.position.x, offset_pose.pose.position.y, offset_pose.pose.position.z);
        
    //     if (!arm_group.setPoseTarget(offset_pose)) {
    //         RCLCPP_ERROR(this->get_logger(), "Invalid offset pose (no IK solution).");
    //         return;
    //     }
    //     arm_group.move();
        
    //     // Now move 10 cm above cube (absolute pose from Gazebo)
    //     auto response = future.get();
    //     auto cube_pose = response->state.pose;
        
    //     geometry_msgs::msg::PoseStamped cube_target_pose;
    //     cube_target_pose.pose = cube_pose;
    //     cube_target_pose.pose.position.z += 0.10;  // 10 cm above
    //     cube_target_pose.pose.orientation = current_pose.pose.orientation;
    //     cube_target_pose.header.frame_id = "base_footprint";
    //     cube_target_pose.header.stamp = this->now();
        
    //     RCLCPP_INFO(this->get_logger(), "Moving above cube at x=%.2f, y=%.2f, z=%.2f",
    //         cube_target_pose.pose.position.x,
    //         cube_target_pose.pose.position.y,
    //         cube_target_pose.pose.position.z);
        
    //     if (!arm_group.setPoseTarget(cube_target_pose)) {
    //         RCLCPP_ERROR(this->get_logger(), "Invalid target pose (no IK solution).");
    //         return;
    //     }
    //     // arm_group.move();
        

    // }

    void get_cube_pose_and_move() {
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
    
        // Pre-grasp: Move 10cm above cube
        geometry_msgs::msg::PoseStamped pre_grasp_pose;
        pre_grasp_pose.header.frame_id = "base_footprint";
        pre_grasp_pose.header.stamp = this->now();
        pre_grasp_pose.pose = cube_pose;
        pre_grasp_pose.pose.position.z += 0.10;
        pre_grasp_pose.pose.orientation.w = 1.0;  // Neutral orientation
    
        // Grasp: Move directly to cube (lower)
        geometry_msgs::msg::PoseStamped grasp_pose = pre_grasp_pose;
        grasp_pose.pose.position.z -= 0.08;  // Drop closer to object
    
        // Place: Offset location
        geometry_msgs::msg::PoseStamped place_pose;
        place_pose.header.frame_id = "base_footprint";
        place_pose.header.stamp = this->now();
        place_pose.pose.position.x = 0.5;
        place_pose.pose.position.y = -0.3;
        place_pose.pose.position.z = 0.85;
        place_pose.pose.orientation.w = 1.0;
    
        // Open gripper
        std::vector<double> open_grip = {0.04, 0.04};
        gripper_group.setJointValueTarget(open_grip);
        gripper_group.move();
    
        // Pre-grasp
        arm_group.setPoseTarget(pre_grasp_pose);
        arm_group.move();
    
        // Grasp
        arm_group.setPoseTarget(grasp_pose);
        arm_group.move();
    
        // Close gripper
        std::vector<double> close_grip = {0.0, 0.0};
        gripper_group.setJointValueTarget(close_grip);
        gripper_group.move();
    
        // Lift object
        arm_group.setPoseTarget(pre_grasp_pose);
        arm_group.move();

        // Grasp (lower to just above cube)
        // grasp_pose.pose.position.z = cube_pose.position.z + 0.015;  // Closer to the object
        // arm_group.setPoseTarget(grasp_pose);
        // arm_group.move();

        // // Close gripper
        // std::vector<double> close_grip = {0.0, 0.0};
        // gripper_group.setJointValueTarget(close_grip);
        // gripper_group.move();

        // // Lift: go higher than pre-grasp
        // geometry_msgs::msg::PoseStamped lift_pose = grasp_pose;
        // lift_pose.pose.position.z += 0.15;  // Lift higher to avoid dragging
        // arm_group.setPoseTarget(lift_pose);
        // arm_group.move();

    
        // Move to place
        arm_group.setPoseTarget(place_pose);
        arm_group.move();
    
        // Open gripper to release
        gripper_group.setJointValueTarget(open_grip);
        gripper_group.move();
    
        RCLCPP_INFO(this->get_logger(), "Pick and place operation complete.");
    }
    

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<PickCubeWithMoveIt>(node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}