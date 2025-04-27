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
        
        // std::vector<double> arm_torso_joint_values = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // std::vector<double> arm_torso_joint_values = {0.3, 2.58, 0.37, 1.07, 1.27, -1.69, 1.36, -1.31};
        // std::vector<double> arm_torso_joint_values = {0.3, 2.58, 0.68, 1.08, 1.61, -1.95, 1.17, -1.3};
        std::vector<double> arm_torso_joint_values = {0.34, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0};
        // std::vector<double> arm_torso_joint_values = {0.3, 0.37, 0.54, -2.5, 2.1, -0.49, 1.4, -1.92};
        std::vector<std::vector<double>> joint_sequences = {
        //   {0.34, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0},
        //   {0.34, 0.10, 0.47, -0.20, 1.56, -1.58, 0.25, 0.0},
        //   {0.34, 0.10, 0.47, -0.20, 1.56, 1.60, 0.25, 1.19},
        //   {0.34, 0.10, 0.47, -0.20, 1.56, 1.60, 0.25, 1.19},
        //   {0.34, 0.78, 0.92, 0.28, 1.7, 1.62, -0.10, 1.66},
        //   {0.34, 1.47, 0.99, 0.78, 1.78, 1.67, -0.24, 1.92},
          {0.34, 0.49, -0.4, -2.56, 1.97, 0.63, -1.03, -0.44},
        //   {0.34, 0.37, 0.54, -2.5, 2.1, -0.49, -1.36, -1.92} //Pose above the cube
        //   {0.34, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

        };
        
        for (size_t i = 0; i < joint_sequences.size(); ++i) {
            arm_torso_group.setJointValueTarget(joint_sequences[i]);
        
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_torso_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                arm_torso_group.move();
                RCLCPP_INFO(this->get_logger(), "Moved to joint target %ld.", i);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to plan to joint target %ld.", i);
                break;  // Optional: stop if one fails
            }
        }

        geometry_msgs::msg::PoseStamped current_pose = arm_group.getCurrentPose();
        std::string reference_frame = arm_group.getPlanningFrame();
        RCLCPP_INFO(this->get_logger(), "Current EE Pose is relative to frame: %s", reference_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "Current EE Pose:");
        RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);

        current_pose.pose.position.y += 0.001;

        auto response = future.get();
        auto cube_pose = response->state.pose;

        RCLCPP_INFO(this->get_logger(), "Moving to cube pose at x=%.2f, y=%.2f, z=%.2f",
                    cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);

        // Open gripper first
        std::vector<double> open_grip = {0.04, 0.04};  // Adjust based on your gripper
        gripper_group.setJointValueTarget(open_grip);
        gripper_group.move();

        // Move to cube pose with offset
        // geometry_msgs::msg::Pose target_pose = pose;
        // // target_pose.position.z -= 0.2; // approach above the cube
        // target_pose.orientation = current_pose.pose.orientation;

        // geometry_msgs::msg::PoseStamped new_current_pose = arm_group.getCurrentPose();
        // geometry_msgs::msg::PoseStamped target_pose;
        // target_pose.pose = pose;
        // target_pose.header.frame_id = "gripper_grasping_frame";
        // target_pose.pose.orientation = new_current_pose.pose.orientation;
        // target_pose.pose.position.z -= 0.10;  // Raise 10 cm above the cube
        // target_pose.pose.orientation.x = 0.0;
        // target_pose.pose.orientation.y = 0.0;
        // target_pose.pose.orientation.z = 0.0;
        // target_pose.pose.orientation.w = 1.0;
        // RCLCPP_INFO(this->get_logger(), "Target Pose: x=%.2f, y=%.2f, z=%.2f",
        //     target_pose.position.x, target_pose.position.y, target_pose.position.z);
        // RCLCPP_INFO(this->get_logger(), "Target Pose: x=%.2f, y=%.2f, z=%.2f",
        //     target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

        // 1. Get current pose
        geometry_msgs::msg::PoseStamped current_pose = arm_group.getCurrentPose();

        // 2. Define offset in local frame
        tf2::Vector3 offset_local(0.0, 0.0, -0.10);  // Move 10 cm down in tool frame

        // 3. Get orientation as tf2 quaternion
        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        );

        // 4. Rotate offset into base frame
        tf2::Vector3 offset_base = tf2::quatRotate(q, offset_local);

        // 5. Apply offset
        geometry_msgs::msg::PoseStamped target_pose = current_pose;
        target_pose.pose.position.x += offset_base.x();
        target_pose.pose.position.y += offset_base.y();
        target_pose.pose.position.z += offset_base.z();

        // 6. Set target and move
        RCLCPP_INFO(this->get_logger(), "Target Pose: x=%.2f, y=%.2f, z=%.2f",
            target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

        if (!arm_group.setPoseTarget(target_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid target pose (no IK solution).");
            return;
        }

        // if (!arm_group.setPoseTarget(target_pose)) {
        //   RCLCPP_ERROR(this->get_logger(), "Invalid target pose (no IK solution).");
        //   return;
        // }

        // arm_group.clearPoseTargets();
        // arm_group.setStartStateToCurrentState();
        // arm_group.setPoseTarget(current_pose);

        // moveit::planning_interface::MoveGroupInterface::Plan plan;
        // if (arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        //     arm_group.move();
        //     RCLCPP_INFO(this->get_logger(), "Moved to current pose.");
        //     RCLCPP_INFO(this->get_logger(), "MoveIt Planning Frame for current_pose: %s", arm_group.getPlanningFrame().c_str());
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to plan to current pose.");
        // }

        // arm_group.clearPoseTargets();
        arm_group.setStartStateToCurrentState();
        // arm_group.setPoseTarget(target_pose);

        // moveit::planning_interface::MoveGroupInterface::Plan plan2;
        // if (arm_group.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        //     arm_group.move();
        //     RCLCPP_INFO(this->get_logger(), "Moved to target pose.");
        //     RCLCPP_INFO(this->get_logger(), "MoveIt Planning Frame for target_pose: %s", arm_group.getPlanningFrame().c_str());
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to plan to target pose.");
        // }


        // Get the current pose after moving to target
        // geometry_msgs::msg::PoseStamped new_current_pose = arm_group.getCurrentPose();
        geometry_msgs::msg::PoseStamped target_pose = cube_pose;
        target_pose.position.z += 0.10;  // 10 cm ABOVE the cube
        
        // Optionally use your robot's current orientation
        geometry_msgs::msg::PoseStamped current_pose = arm_group.getCurrentPose();
        target_pose.pose.orientation = current_pose.pose.orientation;
        
        // 3. Wrap into PoseStamped and assign correct frame (same as reference_frame used in request)
        geometry_msgs::msg::PoseStamped stamped_target_pose;
        stamped_target_pose.pose = target_pose;
        stamped_target_pose.header.frame_id = "base_footprint";  // Same as used in GetEntityState
        stamped_target_pose.header.stamp = this->now();
        
        // 4. Log for confirmation
        RCLCPP_INFO(this->get_logger(), "Moving above cube at x=%.2f, y=%.2f, z=%.2f",
            target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        
        // 5. Set and move
        if (!arm_group.setPoseTarget(stamped_target_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid target pose (no IK solution).");
            return;
        }
        arm_group.move();

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