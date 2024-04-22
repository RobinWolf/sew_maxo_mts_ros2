# gazebo_enviroment
gazebo_enviroment for simulation mobile base and 6DoF kooperation possibilities

## useful links:
UR-Gazebo implementation: https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/tree/humble


## commands:
launch gazebo (empty world): ros2 launch gazebo_ros gazebo.launch.py
convert urdf files to sdf and spawn: ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity <name>


## launch-file:
needed nodes:
- robot_state publisher (use_sim_time:=true)
- gazebo
- spawn_entity 
- control or moveit
