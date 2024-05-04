# gazebo_enviroment
gazebo_enviroment for simulation mobile base and 6DoF kooperation possibilities

## useful links:
UR-Gazebo implementation: https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/tree/humble
Gazebo Package ROS2: https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2
How To create new gazebo model (shelf with KLT, ...): https://classic.gazebosim.org/tutorials?tut=build_model
list of gazebo materials: http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
moveit sourcecode:https://github.com/ros-planning/moveit2/tree/main

## commands:
launch gazebo (empty world): ros2 launch gazebo_ros gazebo.launch.py
convert urdf files to sdf and spawn: ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity <name>


## launch-file:
needed nodes:
- robot_state publisher (use_sim_time:=true) ?
- gazebo
- spawn_entity 
- control or moveit

static_world_launch:
- nodes: gazebo (server and client)
- ros2 launch gazebo_testenviroment static_world.launch.py

empty_world_launch (to develop new worlds and save them in the src/worlds dir.):
- ros2 launch gazebo_ros gazebo.launch.py
