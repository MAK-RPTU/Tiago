from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the Xacro file
    urdf_file = "/home/javier/Desktop/KK/Isaac_Tutorials/tiago_ws/src/tiago_robot/tiago_description/robots/tiago.urdf.xacro"

    # Declare the `use_sim_time` launch argument (default: true)
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time if true"
    )

    # Ensure `use_sim_time` is set correctly
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Define robot_state_publisher Node
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[{
    #         "robot_description": Command(["xacro ", urdf_file]),
    #         "use_sim_time": use_sim_time  # Ensure simulation time is enabled
    #     }]
    # )

    # Define the `joint_space` MoveIt node (your C++ script)
    # joint_space_node = Node(
    #     package="cartesian_mover",  # Change this to your actual package name
    #     executable="joint_space_simple",
    #     output="screen",
    #     parameters=[{
    #         "use_sim_time": use_sim_time  # Ensure time sync with simulation
    #     }]
    # )

    # joint_space_node = Node(
    #     package="cartesian_mover",  # Change this to your actual package name
    #     executable="joint_space_scalable",
    #     output="screen",
    #     parameters=[{
    #         "use_sim_time": use_sim_time  # Ensure time sync with simulation
    #     }]
    # )


    return LaunchDescription([
        # use_sim_time_arg,
        # robot_state_publisher,
        # joint_space_node
        Node(
            package='cartesian_mover',
            executable='task_executor_server',
            name='task_executor_server',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
        
    ])
