#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="false")
    
    # tetra Motor Driver Board
    tetra_node = Node(
        package='tetra', 
        executable='tetra',
        output='screen',
        parameters=[
            {"m_bEKF_option": True} 
        ]
    )
    
    # EKF Localization
    ekf_localization_node= Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("tetra_bringup"), 'params', 'ekf.yaml')],
        arguments=['--ros-args', '--log-level', 'error']
    )
    
    # tetra_interface Board
    tetra_interface_node = Node(
        package='tetra_interface', 
        executable='tetra_interface',
        output='screen',
        parameters=[
            {"m_bConveyor_option": False},
            {"m_bUltrasonic_option": False}
        ]
    )
    
    # IMU Sensor
    iahrs_driver_node = Node(
        package='iahrs_driver', 
        executable='iahrs_driver',
        output='screen',
        parameters=[
            {"m_bSingle_TF_option": False} 
        ]
    )
    
    # Joystick
    joy_node = Node(
        package='joy', 
        executable='joy_node',
        output='screen',
        parameters=[
            {"deadzone": 0.05}
        ]
    )
    
    # tetra_URDF
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('tetra_description'),
                        'urdf',
                        'tetra.xacro',
                    ]),
                ]),
        }]
    )
    
    # tetra_service
    tetra_service_node = Node(
        package='tetra_service', 
        executable='tetra_service',
        respawn= True,
        output='screen',
        parameters=[
            {"m_dHome_ID": 0}
        ]
    )

    # rosbridge_server 
    rosbridge_server = Node(
        package='rosbridge_server', 
        executable='rosbridge_websocket',
        output='screen',
        parameters=[
            {"port": 9090}
        ]
    )

    # rosapi_node 
    rosapi_node = Node(
        package='rosapi', 
        executable='rosapi_node',
        output='screen',
    )
    
    # Sensor Launch Files
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']
        )
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('livox_ros_driver2'), '/launch_ROS2/MID360_launch.py']
        )
    )
    
    cyglidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('cyglidar_d2_ros2'), '/launch/cyglidar.launch.py']
        )
    )

    # Apriltag Node 
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('/image_rect', '/camera1/color/image_raw'), 
            ('/camera_info', '/camera1/color/camera_info') 
        ]
    )
    
    #pointcloud to laserscan Node
    converter_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_converter',
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/livox/laser_scan')
        ],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 2.0,
            'range_min': 0.1,
            'range_max': 8.0,
            'angle_min': -3.14159, # -180deg
            'angle_max': 3.14159,  # +180deg
            'angle_increment': 0.0087, # 0.5deg
            'concurrency_level': 1,
        }]
    )
    
    # create and return launch description object
    return LaunchDescription(
        [
            # Launch Arguments
            use_sim_time,
            
            # Platform Nodes...
            tetra_node,
            ekf_localization_node,
            tetra_interface_node, 
            iahrs_driver_node,
            joy_node,
            rsp_node,
            tetra_service_node,
            rosbridge_server,
            rosapi_node,
            
            # Sensor Launch Files
            realsense_launch,
            livox_launch,
            cyglidar_launch,
            
            # pointcloud to laserscan
            converter_node,
            
            # Apriltag 
            apriltag_node,
        ]
    )
