# sew_agv_drivers

This Repo contains ROS2 code to controll a SEW AGV. The communication to the AGV is based on the Repo LinkTODO by Tom Schneider.

## Helpful informations:
ROS Control: https://control.ros.org/humble/doc/ros2_control/doc/index.html 

Diffdrive Repo by ArticulatedRobotics: https://github.com/joshnewans/diffdrive_arduino/tree/humble 


## Helpful comands
### Launch single packages
Launch driver:
```
ros2 launch sew_agv_drivers launch_agv.launch.py rviz:=true use_fake_hardware:=true
```
Move agw via terminal:
```
ros2 topic pub /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}" --once
```
Move agv via joystick:
```
ros2 launch sew_agv_navigation joystick.launch.py
```
### Launch all together with gazebo or real hardware
```
ros2 launch sew_agv_navigation bringup.launch.py
```
```
ros2 launch sew_agv_navigation navigation.launch.py
```

## TODOs 
- 
