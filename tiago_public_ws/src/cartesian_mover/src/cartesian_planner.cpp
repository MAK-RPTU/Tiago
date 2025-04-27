// Include necessary ROS 2 and MoveIt libraries
#include <rclcpp/rclcpp.hpp>                    // Core ROS 2 C++ client library for nodes, logging, etc.
#include <pluginlib/class_loader.hpp>           // Pluginlib library for loading planner plugins
// #include <moveit/robot_model_loader/robot_model_loader.hpp>  // Loads robot model from ROS parameter server
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>  // Interface for planner plugins
#include <moveit/planning_scene/planning_scene.h>          // Planning scene to maintain world state
#include <moveit/kinematic_constraints/utils.h>            // Utilities for constructing kinematic constraints
#include <moveit/move_group_interface/move_group_interface.h>  // MoveIt interface for motion planning and execution

static const rclcpp::Logger LOGGER = rclcpp::get_logger("cartesian_planner");  // Define a logger for this node

int main(int argc, char** argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);                   // Start the ROS 2 communication layer with command-line arguments

    // Create node options with automatic parameter declaration
    rclcpp::NodeOptions node_options;           // Create options object for node configuration
    node_options.automatically_declare_parameters_from_overrides(true);  // Allow parameter overrides from command line

    // Create a shared pointer to the node
    std::shared_ptr<rclcpp::Node> move_group_node =
        rclcpp::Node::make_shared("cartesian_planner", node_options);  // Create a node named "move_group_tutorial"

    // Start an executor in a separate thread (minimal usage here, as we won't spin extensively)
    rclcpp::executors::SingleThreadedExecutor executor;  // Create a single-threaded executor
    executor.add_node(move_group_node);                 // Add the node to the executor
    std::thread([&executor]() { executor.spin(); }).detach();  // Spin the executor in a background thread

    // Define the planning group name
    const std::string PLANNING_GROUP = "arm";  // Set the planning group to "panda_arm" (adjust if different)

    // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader(move_group_node, "robot_description");  // Load robot model from "robot_description" parameter
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();  // Get the robot model pointer

    // Create a robot state to track the current pose
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));  // Initialize robot state
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);  // Get joint model group for the planning group

    // Create a planning scene
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));  // Initialize planning scene

    // Configure a valid initial robot state
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");  // Set to a default "ready" state

    // Load the planner plugin
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;  // Unique pointer for plugin loader
    planning_interface::PlannerManagerPtr planner_instance;  // Pointer to the planner instance
    std::vector<std::string> planner_plugin_names;           // Vector to store planner plugin names

    // Get planner plugin names from parameters
    if (!move_group_node->get_parameter("ompl.planning_plugins", planner_plugin_names))  // Try to get planner plugin names
    {
        RCLCPP_FATAL(LOGGER, "Could not find planner plugin names");  // Log fatal error if not found
        return -1;  // Exit with error code
    }

    // Initialize the plugin loader
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));  // Create plugin loader for MoveIt planners
    }
    catch (pluginlib::PluginlibException& ex)
    {
        RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());  // Log loader exception
        return -1;  // Exit with error code
    }

    // Check if planner plugins are available
    if (planner_plugin_names.empty())
    {
        RCLCPP_ERROR(LOGGER, "No planner plugins defined. Please ensure planning_plugins parameter is set.");  // Log error if no plugins
        return -1;  // Exit with error code
    }

    // Load the first planner plugin
    const auto& planner_name = planner_plugin_names.at(0);  // Use the first plugin name
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_name));  // Create planner instance
        if (!planner_instance->initialize(robot_model, move_group_node, move_group_node->get_namespace()))  // Initialize planner
        {
            RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");  // Log initialization failure
            return -1;  // Exit with error code
        }
        RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());  // Log planner name
    }
    catch (pluginlib::PluginlibException& ex)
    {
        RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s", planner_name.c_str(), ex.what());  // Log loading exception
        return -1;  // Exit with error code
    }

    // Create MoveGroupInterface for execution
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);  // Initialize MoveGroupInterface

    // Set the reference frame for pose targets
    move_group.setPoseReferenceFrame("base_footprint");  // Set reference frame to "panda_link0" (adjust if different)

    // Set the end effector link
    move_group.setEndEffectorLink("gripper_grasping_frame");  // Set end effector to "panda_hand_tcp" (adjust if different)

    // Define a pose goal
    planning_interface::MotionPlanRequest req;        // Create a motion plan request
    planning_interface::MotionPlanResponse res;       // Create a motion plan response
    geometry_msgs::msg::PoseStamped pose;            // Create a pose stamped message
    pose.header.frame_id = "gripper_grasping_frame";            // Set the frame ID to the reference frame
    pose.pose.position.x = -0.2;                      // Set X position (0.3m)
    pose.pose.position.y = 0.0;                      // Set Y position (0.0m)
    pose.pose.position.z = 0.0;                      // Set Z position (0.5m)
    pose.pose.orientation.w = 1.0;                   // Set orientation to identity (no rotation)

    // Define tolerances for the pose goal
    std::vector<double> tolerance_pose(3, 0.01);     // Set position tolerance to 0.01m in all axes
    std::vector<double> tolerance_angle(3, 0.01);    // Set orientation tolerance to 0.01 radians in all axes

    // Construct the pose goal constraint
    moveit_msgs::msg::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("gripper_grasping_frame", pose, tolerance_pose, tolerance_angle);  // Create constraint

    // Populate the motion plan request
    req.group_name = PLANNING_GROUP;                 // Set the planning group
    req.goal_constraints.push_back(pose_goal);       // Add the pose goal constraint

    // Define workspace bounds (optional, but included for completeness)
    req.workspace_parameters.min_corner.x = -5.0;    // Set minimum X bound
    req.workspace_parameters.min_corner.y = -5.0;    // Set minimum Y bound
    req.workspace_parameters.min_corner.z = -5.0;    // Set minimum Z bound
    req.workspace_parameters.max_corner.x = 5.0;     // Set maximum X bound
    req.workspace_parameters.max_corner.y = 5.0;     // Set maximum Y bound
    req.workspace_parameters.max_corner.z = 5.0;     // Set maximum Z bound

    // Create a planning context
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);  // Get planning context

    // Check if context creation was successful
    if (!context)
    {
        RCLCPP_ERROR(LOGGER, "Failed to create planning context");  // Log error if context is null
        return -1;  // Exit with error code
    }

    // Solve the planning problem
    context->solve(res);                             // Compute the plan
    if (res.error_code_.val != res.error_code_.SUCCESS)  // Check if planning succeeded
    {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");  // Log error if failed
        return -1;  // Exit with error code
    }

    // Execute the plan using MoveGroupInterface
    moveit_msgs::msg::MotionPlanResponse response;    // Create response message
    res.getMessage(response);                        // Get the planned trajectory
    move_group.execute(response.trajectory);         // Execute the trajectory

    RCLCPP_INFO(LOGGER, "Motion executed successfully");  // Log success message

    // Update the planning scene with the final state
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);  // Set final joint positions
    planning_scene->setCurrentState(*robot_state.get());  // Update planning scene

    // Shutdown the ROS 2 node
    rclcpp::shutdown();                             // Clean up and shut down the ROS 2 communication layer

    return 0;                                       // Return success code
}