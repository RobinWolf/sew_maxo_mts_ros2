from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    description_package = "sew_agv_description"
    sim_package = "gazebo_testenviroment"
    driver_package = "sew_agv_drivers"
    navigation_package = "sew_agv_navigation"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='sew_',
            description="Prefix for the links and joints in the robot cell",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "standalone_gazebo",
            default_value='true',
            description="add the robot description to gazebo with a simpler approach, using a diff_drive and lidar plugin",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_with_gazebo",
            default_value='false',
            description="add the robot description to gazebo ros2 control for the diff_drive, no gazebo internal plugin!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='false',
            description="launch the drivers that connect to the real hardware via IP",
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('world',
            default_value='src/gazebo_testenviroment/worlds/static_world_2404.world',
            description='Specify the world which should be loaded in Gazebo'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value='TODO',
            description="the IP the real robot can be pinged",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_joystick",
            default_value='true',
            description="set to true if you want to use a joystick (XBox controller) to move the robot",
        )
    )


    #init launch arguments, transfer to variables
    world = LaunchConfiguration('world')        
    tf_prefix = LaunchConfiguration("tf_prefix")
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")
    use_sim_time = LaunchConfiguration('use_sim_time')   
    generate_ros2_control_tag = LaunchConfiguration('generate_ros2_control_tag') 
    ros2_control_with_gazebo = LaunchConfiguration("ros2_control_with_gazebo")
    robot_ip = LaunchConfiguration('robot_ip') 
    enable_joystick = LaunchConfiguration('enable_joystick') 


    #lauch gazebo if launch argument is set to true
    load_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(sim_package), 'launch']), "/robot.launch.py"]),
            condition=IfCondition(standalone_gazebo),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "world": world,
                "standalone_gazebo": standalone_gazebo,
                'launch_rviz':'false',
            }.items(),
    )

  
    #launch real drivers if launch argument is set to true
    load_real_drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(driver_package), 'launch']), "/driver.launch.py"]),
            condition=IfCondition(generate_ros2_control_tag),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "robot_ip": robot_ip,
                "generate_ros2_control_tag": generate_ros2_control_tag,
                "ros2_control_with_gazebo": ros2_control_with_gazebo,
                "use_sim_time": use_sim_time,
            }.items(),
    )


    #launch real sensors such as lidar --> currently not feasable dur to lack in hardware support
    #load_lidar = 

    #launch the joystick
    load_joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(navigation_package), 'launch']), "/joystick.launch.py"]),
            condition=IfCondition(enable_joystick),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
    )



    nodes_to_start = [
        load_gazebo,
        load_joystick,
        load_real_drivers
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
