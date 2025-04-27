#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>   // <-- important
#include <thread>

class TiagoPickPlace
{
public:
  explicit TiagoPickPlace(const rclcpp::Node::SharedPtr &node) : node_(node)
  {
    move_group_arm_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_torso");
    move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper");

    move_group_arm_torso_->setMaxVelocityScalingFactor(0.5);
    move_group_arm_torso_->setMaxAccelerationScalingFactor(0.5);
    move_group_arm_torso_->setPlanningTime(10.0);

    // Create service client for /gazebo/get_entity_state
    entity_state_client_ = node_->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
  }

  void moveToHomePosition()
  {
    std::vector<double> home_position = {0.34, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group_arm_torso_->setJointValueTarget(home_position);
    executeMovement(*move_group_arm_torso_, "Moved to home position successfully.", "Failed to move to home position.");
  }

  void moveToPregraspPosition()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.497;
    target_pose.position.y = -0.564;
    target_pose.position.z = 1.127;
    target_pose.orientation.w = 1.0;

    move_group_arm_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_torso_, "Moved to pregrasp position successfully.", "Failed to move to pregrasp position.");
  }

  void approachObject(double distance)
  {
    geometry_msgs::msg::Pose start_pose = move_group_arm_torso_->getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    start_pose.position.z -= distance;
    waypoints.push_back(start_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_arm_torso_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction == 1.0)
    {
      executeTrajectory(*move_group_arm_torso_, trajectory, "Approach executed successfully.", "Failed to execute approach.");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to compute approach path.");
    }
  }

  void controlGripper(const std::string &action)
  {
    std::vector<double> grip_positions;
  
    if (action == "close")
    {
      grip_positions = {0.02, 0.02};  // Closed
    }
    else if (action == "open")
    {
      grip_positions = {0.0, 0.0};  // Opened
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Unknown gripper command: %s", action.c_str());
      return;
    }
  
    move_group_gripper_->setJointValueTarget(grip_positions);
    executeMovement(*move_group_gripper_, action + " gripper!", "Failed to " + action + " gripper!");
  }
  
  void retreatObject(double distance)
  {
    geometry_msgs::msg::Pose start_pose = move_group_arm_torso_->getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    start_pose.position.z += distance;
    waypoints.push_back(start_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_arm_torso_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction == 1.0)
    {
      executeTrajectory(*move_group_arm_torso_, trajectory, "Retreat executed successfully.", "Failed to execute retreat.");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to compute retreat path.");
    }
  }

  void moveToPlacePosition()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.4;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.3;
    target_pose.orientation.w = 1.0;

    move_group_arm_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_torso_, "Moved to place position successfully.", "Failed to move to place position.");
  }

  void moveToArucoCube()
  {
    if (!entity_state_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Service /gazebo/get_entity_state not available.");
      return;
    }

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = "aruco_cube";
    request->reference_frame = "base_footprint";

    auto future = entity_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call /gazebo/get_entity_state service.");
      return;
    }

    auto response = future.get();
    auto cube_pose = response->state.pose;

    geometry_msgs::msg::Pose target_pose = cube_pose;
    target_pose.position.z += 0.1;  // Offset a little above
    target_pose.orientation.w = 1.0; // Facing front

    move_group_arm_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_torso_, "Moved towards Aruco cube successfully.", "Failed to move towards Aruco cube.");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_torso_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr entity_state_client_;  // <-- Service client

  void executeMovement(moveit::planning_interface::MoveGroupInterface &group, const std::string &success_msg, const std::string &fail_msg)
  {
    group.setStartStateToCurrentState();
    if (group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "%s", success_msg.c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "%s", fail_msg.c_str());
    }
  }

  void executeTrajectory(moveit::planning_interface::MoveGroupInterface &group, const moveit_msgs::msg::RobotTrajectory &trajectory, const std::string &success_msg, const std::string &fail_msg)
  {
    group.setStartStateToCurrentState(); 
    if (group.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "%s", success_msg.c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "%s", fail_msg.c_str());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tiago_pick_place");
  auto app = std::make_shared<TiagoPickPlace>(node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  app->moveToHomePosition();
  app->moveToPregraspPosition();
  app->moveToArucoCube();
  app->approachObject(0.05);
  app->controlGripper("close");
  app->retreatObject(0.1);
  app->moveToPlacePosition();
  app->controlGripper("open");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
