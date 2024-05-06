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
    
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Set to "true" if you want to use the joystick with gazebo, set to "fasle" if you use real hardware.'))
    

    #init launch arguments, transfer to variables
    world = LaunchConfiguration('world')        
    tf_prefix = LaunchConfiguration("tf_prefix")
    ros2_control_with_gazebo = LaunchConfiguration("ros2_control_with_gazebo")  # --> maybe not necessary
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")
    use_sim_time = LaunchConfiguration('use_sim_time')   


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "sew_agv_model.urdf.xacro"]),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "ros2_control_with_gazebo:=",
            ros2_control_with_gazebo,
            " ",
            "standalone_gazebo:=",
            standalone_gazebo,
        ]
    )

    #convert robot_description_content to string, thet it can be passed as yaml to the robot_state_publisher Node --> unsafe without !
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str), 'use_sim_time': True} 
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    #lauch gazebo (empty world)
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch']), '/gazebo.launch.py']),
    )
    

    #spawn the robot into the empty world (establish a bridge between ros2 topics like cmd_vel and gazebo)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'sew-maxo-mts-agv'],
        output='screen'
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare(sim_package), "rviz", "gazebo_teleop_lidar_rviz.rviz"]) # define path to rviz-config file

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )


    nodes_to_start = [
        gazebo_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)