from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    description_package = "sew_agv_description"
    sim_package = "gazebo_testenviroment"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix for the links and joints in the robot cell",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_with_gazebo",
            default_value='false',
            description="add the robot description to gazebo using a ros2_control hardware interface",
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
    DeclareLaunchArgument('world', default_value='src/gazebo_testenviroment/worlds/static_world_2404.world',
                            description='Specify the world which should be loaded in Gazebo'))
    

    #init launch arguments, transfer to variables
    world = LaunchConfiguration('world')        
    tf_prefix = LaunchConfiguration("tf_prefix")
    ros2_control_with_gazebo = LaunchConfiguration("ros2_control_with_gazebo")
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")

    gazebo_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(sim_package), 'launch']), "/robot.launch.py"]),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "world": world,
                "ros2_control_with_gazebo": ros2_control_with_gazebo,
                "standalone_gazebo": standalone_gazebo,
            }.items(),
    )

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{'enable_button': 2}]  # enable teleop of the robot by pressing X on the xBox controller
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}]  # Specify the device file for your joystick
    )


    nodes_to_start = [gazebo_robot_node, teleop_joy_node, joy_node]


    return LaunchDescription(declared_arguments + nodes_to_start)