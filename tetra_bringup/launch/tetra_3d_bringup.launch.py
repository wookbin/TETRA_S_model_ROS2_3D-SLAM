#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


# this is the function launch  system will look for
def generate_launch_description():

    # tetra Motor Driver Board
    tetra_node = Node(
        package='tetra', 
        executable='tetra',
        output='screen',
        parameters=[
            {"m_bEKF_option": True} #default: False
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
            {"m_bSingle_TF_option": False} # default: True
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
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="false")
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
    
    #keep-alive: image_raw를 상시 구독/재발행 (초경량)
    keepalive_relay = Node(
        package='topic_tools',
        executable='relay',
        name='image_keepalive_relay',
        output='screen',
        arguments=['/camera1/color/image_raw', '/camera1/color/image_raw_passthrough']
    )
    
    ## depthimage to laserscan
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='realsense_depth_to_scan',
        remappings=[
            ('depth', '/camera1/depth/image_rect_raw'),
            ('depth_camera_info', '/camera1/depth/camera_info'),
            ('scan', '/camera1/depth/scan')
        ],
        parameters=[os.path.join(get_package_share_directory("tetra_bringup"), 'params', 'realsense_depth_to_scan.yaml')],
        output='screen'
    )
    
    # create and return launch description object
    return LaunchDescription(
        [
            tetra_node,
            ekf_localization_node,
            tetra_interface_node, 
            iahrs_driver_node,
            joy_node,
            use_sim_time,
            rsp_node,
            tetra_service_node,
            rosbridge_server,
            rosapi_node,
            keepalive_relay,
            depth_to_scan_node,
            
        
        ## apriltag_ros
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('apriltag_ros'), '/launch/apriltag_detection.launch.py']),
		),
			
		## sick_tim_571
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('sick_scan_xd'), '/launch/sick_tim_5xx.launch.py']),
		),
		
		## laser filter (shadow_filter)
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('laser_filters'), '/examples/shadow_filter_example.launch.py']),
		),
		
		## cygbot 2D lidar
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('cyglidar_d2_ros2'), '/launch/cyglidar.launch.py']),
		),
		
		## realsense D455
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
		),

		## Livox MID-360 (3D LiDAR)
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('livox_ros_driver2'), '/launch_ROS2/MID360_launch.py']),
		),
		
		# costmap_to_cloud (new add..)
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        [get_package_share_directory('tetra_costmap_to_cloud'), '/launch/costmap_to_cloud.launch.py']),
        #),
        
        ]
    )
