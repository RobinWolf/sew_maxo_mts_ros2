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
ros2 launch sew_agv_navigation navigation.launch.py launch_rviz:=true
```






## Hardware Interface
### Setup connection to AGV:
```
sudo ip addr add 192.168.10.222/24 dev enx9cebe8ea463d
sudo ip addr show enx9cebe8ea463d
```
Here `enx9cebe8ea463d` is the Ethernet interface connected to your network. To find out type: `ip addr show`. Something like this shows up:
```
4: enx9cebe8ea463d: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether 9c:eb:e8:ea:46:3d brd ff:ff:ff:ff:ff:ff
```

### Launch hardware interface
```
ros2 launch sew_agv_drivers launch_agv.launch.py 
```



## TODOs 
- 
