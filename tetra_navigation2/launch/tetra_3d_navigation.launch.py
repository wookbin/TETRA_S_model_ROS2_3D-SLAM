import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_navigation = get_package_share_directory('tetra_navigation2')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Flag to enable use_sim_time'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'launch',
        'navigation_launch.py'
    )

    localization_params_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'params',
        'amcl_localization.yaml'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'params',
        'navigation.yaml'
    )

    virtual_wall_launch_path = os.path.join(
        get_package_share_directory('virtual_wall'),
        'launch',
        'virtual_wall.launch.py'
    )
    
    # Dynamically constructed paths
    #map_file_path = PathJoinSubstitution([
    #    '/home/tetra/ros2_ws/src/tetra_navigation2', 'maps', LaunchConfiguration('map_name')
    #])

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
                #'map': map_file_path,
        }.items()
    )

    virtual_wall_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(virtual_wall_launch_path),
        launch_arguments={}.items()
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(navigation_launch)
    launchDescriptionObject.add_action(virtual_wall_launch)

    return launchDescriptionObject
