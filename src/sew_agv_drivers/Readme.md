# sew_agv_drivers

This Repo contains ROS2 code to controll a SEW AGV. The communication to the AGV is based on the Repo LinkTODO by Tom Schneider.

## Helpful informations:
ROS Control: https://control.ros.org/humble/doc/ros2_control/doc/index.html 

Diffdrive Repo by ArticulatedRobotics: https://github.com/joshnewans/diffdrive_arduino/tree/humble 


## Helpful comands
```
ros2 launch sew_agv_drivers launch_agv.launch.py rviz:=true
```
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once
```

## TODOs 
- Prüfen ob topic auf das diff_drive_controller (im yaml file) hört geändert werden kann (aktuell cmd_vel)  --> damit controller über navigation priorisiert werden kann (mit twist max node)
