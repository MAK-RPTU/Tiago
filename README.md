# Tiago
THis is tiago repository for ROS2

Instructions:

cd ~/tiago_public_ws

`sudo rosdep init`

`rosdep update`

`rosdep install --from-paths src -y --ignore-src`

`source /opt/ros/humble/setup.bash`

`colcon build --symlink-install`

`source ~/tiago_public_ws/install/setup.bash`

To launch gazebo with tabletop_cube:

`ros2 launch tiago_gazebo tiago_gazebo.launch.py moveit:=True is_public_sim:=True world_name:=tabletop_cube use_sim_time:=True`

To launch moveit node:

`ros2 launch tiago_moveit_config moveit_rviz.launch.py use_sim_time:=True`


To launch task executor, (under development):

`ros2 run cartesian_mover pick_cube_node --ros-args -p use_sim_time:=true`