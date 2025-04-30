#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include "attach_gazebo_interfaces/srv/attach_command.hpp"


class TiagoPickPlace
{
public:
  explicit TiagoPickPlace(const rclcpp::Node::SharedPtr &node) : node_(node)
  {
    // if (!node_->has_parameter("use_sim_time")) {
    //   node_->declare_parameter("use_sim_time", true);
    // }
    move_group_arm_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_torso");
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper");

    move_group_arm_torso_->setMaxVelocityScalingFactor(0.5);
    move_group_arm_torso_->setMaxAccelerationScalingFactor(0.5);
    move_group_arm_torso_->setPlanningTime(10.0);
  }

  void callAttachService(const std::string &command)
  {
    auto client = node_->create_client<attach_gazebo_interfaces::srv::AttachCommand>("/attach_control");
  
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Attach service not available.");
      return;
    }
  
    auto request = std::make_shared<attach_gazebo_interfaces::srv::AttachCommand::Request>();
    request->command = command;
  
    auto future = client->async_send_request(request);
  
    // Use the existing executor (no need to add the node again)
  }
  
  


  void addTableToPlanningScene()
  {
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = move_group_arm_torso_->getPlanningFrame();

      collision_object.id = "table1";

      // Define table shape
      shape_msgs::msg::SolidPrimitive table_primitive;
      table_primitive.type = table_primitive.BOX;
      table_primitive.dimensions.resize(3);
      table_primitive.dimensions[0] = 1.0;   // X size (length)
      table_primitive.dimensions[1] = 0.8;   // Y size (width)
      table_primitive.dimensions[2] = 0.83;  // Z size (height)

      // Define table pose
      geometry_msgs::msg::Pose table_pose;
      table_pose.orientation.w = 1.0;
      table_pose.position.x = 0.88;
      table_pose.position.y = 0.0;
      table_pose.position.z = 0.83 - 0.83/2.0; // Center of box: table height - half table thickness
      // table_pose.position.z = 0.0; // Center of box: table height - half table thickness
      collision_object.primitives.push_back(table_primitive);
      collision_object.primitive_poses.push_back(table_pose);
      collision_object.operation = collision_object.ADD;

      planning_scene_interface.applyCollisionObject(collision_object);

      RCLCPP_INFO(node_->get_logger(), "Table added to planning scene!");
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

  void safe_position(const std::vector<double>& joint_angles_input)
  {
      if (joint_angles_input.size() != 8)
      {
          RCLCPP_ERROR(node_->get_logger(), "Input vector must have exactly 8 elements (1 torso + 7 arm joints).");
          return;
      }
  
      std::vector<double> target_joint_values;
  
      // First element: fixed torso value (DO NOT convert)
      target_joint_values.push_back(joint_angles_input[0]);  // Torso stays as meters
  
      // Remaining 7 elements: convert degrees to radians
      for (size_t i = 1; i < joint_angles_input.size(); ++i)
      {
          double angle_deg = joint_angles_input[i];
          double angle_rad = angle_deg * (M_PI / 180.0);
          target_joint_values.push_back(angle_rad);
      }
  
      for (size_t i = 0; i < target_joint_values.size(); ++i)
      {
          RCLCPP_INFO(node_->get_logger(), "Joint[%zu]: %.4f", i, target_joint_values[i]);
      }
  
      move_group_arm_torso_->setJointValueTarget(target_joint_values);
      executeMovement(*move_group_arm_torso_, "Moved to safe position successfully.", "Failed to move to safe position.");
  }
  

  void moveTograspPosition()
  {
      geometry_msgs::msg::Pose cube_pose;
      // cube_pose.position.x = 0.53;
      // cube_pose.position.y = -0.0499;
      // cube_pose.position.z = 0.8644;  // Table height + half cube height
      // cube_pose.orientation.w = 1.0;  // Assume no rotation for now
      cube_pose.position.x = 0.370;
      cube_pose.position.y = -0.043;
      cube_pose.position.z = 1.147;  // Table height + half cube height
      // cube_pose.position.z = 0.869; // Exact centere of cube
      // cube_pose.orientation.x = -0.534;  // Assume no rotation for now
      // cube_pose.orientation.y = 0.464;  // Assume no rotation for now
      // cube_pose.orientation.z = 0.534;  // Assume no rotation for now
      // cube_pose.orientation.w = -0.463;  // Assume no rotation for now

      geometry_msgs::msg::Pose target_pose;
  
      // Copy position
      target_pose.position = cube_pose.position;
      // target_pose.orientation = cube_pose.orientation;
  
      // Adjust z to hover above the cube
      // target_pose.position.z += 0.1;  // Stay 10 cm above
  
      // Set gripper orientation to point down (roll 180 deg)
      // tf2::Quaternion orientation;
      // orientation.setRPY(0.0, 0.0, 0.0);
      // target_pose.orientation = tf2::toMsg(orientation);
  
      // move_group_arm_torso_->setPlanningTime(10.0);  // <-- Important
      // move_group_arm_torso_->setGoalOrientationTolerance(0.05);  // <-- Important

      move_group_arm_torso_->setPoseTarget(target_pose);  // Use tool link!
      executeMovement(*move_group_arm_torso_, "Moved above cube successfully.", "Failed to move above cube.");
  }

  void rotate_ee(double wrist_angle)
  {
      // Get the current joint values
      std::vector<double> current_joint_values = move_group_arm_torso_->getCurrentJointValues();
  
      // Modify only the last joint (e.g., wrist roll joint)
      if (current_joint_values.size() > 0)
      {
          current_joint_values.back() = wrist_angle;  // Set last joint to input angle
      }
      else
      {
          RCLCPP_ERROR(node_->get_logger(), "Joint state vector is empty!");
          return;
      }
  
      // Set new target
      move_group_arm_torso_->setJointValueTarget(current_joint_values);
  
      executeMovement(*move_group_arm_torso_, "Rotated end-effector successfully.", "Failed to rotate end-effector.");
  }
  

  void approachObject(double distance)
  {
    // geometry_msgs::msg::Pose start_pose = move_group_arm_torso_->getCurrentPose().pose;
    geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    start_pose.position.z -= distance;
    waypoints.push_back(start_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    // double fraction = move_group_arm_torso_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    double fraction = move_group_arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction == 1.0)
    {
      // executeTrajectory(*move_group_arm_torso_, trajectory, "Approach executed successfully.", "Failed to execute approach.");
      executeTrajectory(*move_group_arm_, trajectory, "Approach executed successfully.", "Failed to execute approach.");
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
      grip_positions = {0.023, 0.023};  // Closed
    }
    else if (action == "open")
    {
      grip_positions = {0.043, 0.043};  // Opened
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
    // target_pose.position.x = 0.4;
    // target_pose.position.y = -0.2;
    // target_pose.position.z = 0.3;
    // target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.541;
    target_pose.position.y = -0.260;
    target_pose.position.z = 1.13;
    target_pose.orientation.w = 1.0;

    move_group_arm_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_torso_, "Moved to place position successfully.", "Failed to move to place position.");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_torso_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;

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

  // app->addTableToPlanningScene();

  app->moveToHomePosition();
  app->moveToPregraspPosition();
  std::vector<double> joints_with_torso = {0.34, 22.0, 8.0, -122.0, 88.0, 84.0, -70.0, -54.0};
  // Call safe_position function
  app->safe_position(joints_with_torso);

  joints_with_torso = {0.213, 12.0, 26.0, -104.0, 102.0, 75.0, -64.0, -71.0};
  // Call safe_position function
  app->safe_position(joints_with_torso);

  app->controlGripper("open");
  // app->moveTograspPosition();
  // app->rotate_ee(-M_PI / 2.0);
  // app->moveToPregraspPosition();
  app->approachObject(0.08);
  // app->controlGripper("close");

  app->callAttachService("close");

  app->retreatObject(0.08);
  app->moveToPregraspPosition();

  app->moveToPlacePosition();
  app->rotate_ee(-M_PI / 2.0);
  app->approachObject(0.05);
  // app->controlGripper("open");
  app->callAttachService("open");
  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
