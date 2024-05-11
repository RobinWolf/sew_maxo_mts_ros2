from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    param_file_name = 'mapping.yaml' 
    navigation_package = "sew_agv_navigation"

    default_param_dir = os.path.join(get_package_share_directory(navigation_package), 'config', 'navigation' + param_file_name)
    

    declared_arguments = []
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('autostart',
            default_value='true',
            description='Set to "true" if you want to start the nav2 lifecycle automatically in launch.'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('param_dir',
            default_value=default_param_dir,
            description='pass the path to your navigation.yaml file where params for slam_toolbox node are defined.'
        )
    )

    #init launch arguments, transfer to variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')  
    param_dir = LaunchConfiguration('param_dir')     


    #########################################################################################################################
    ###                                            nodes for neo_localization                                             ###
    #########################################################################################################################
    #all params for these nodes are defined in the navigation.yaml file. Every section of this file connects to the drsired node name
    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[param_dir])
    
    lifecycle_nodes_localization = ['map_server']
    lifecycle_manager_localization = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes_localization}]) 
    
    neo_localization2_node = Node(
            package='neo_localization2', 
            executable='neo_localization_node', 
            output='screen',
            name='neo_localization2_node', 
            parameters= [param_dir])
    
    #########################################################################################################################
    ###                                               nodes for neo_navigation                                            ###
    #########################################################################################################################
    controller_server = Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[param_dir])
    planner_server = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[param_dir])
    behavior_server = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[param_dir])
    bt_navigator = Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[param_dir])
    waypoint_follower = Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[param_dir])
    
    lifecycle_nodes_navigation = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']
    lifecycle_manager_navigation = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_navigation}])


    nodes_to_start = [
        map_server,
        lifecycle_manager_localization,
        neo_localization2_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager_navigation,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

#nav2_controller
#nav2_planner
#nav2_behaviors
#nav2_bt_navigator
#nav2_waypoint_follower
#nav2_waypoint_follower

#https://github.com/neobotix/neo_nav2_bringup/blob/rolling/launch/localization_neo.launch.py --> Multi Robor nicht berücksichtigen!
#https://github.com/neobotix/neo_nav2_bringup/blob/rolling/launch/navigation_neo.launch.py

#configs für node aus config übernehmen