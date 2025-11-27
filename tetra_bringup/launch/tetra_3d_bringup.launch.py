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


# this is the function launch system will look for
def generate_launch_description():

    # --- 1. Launch Arguments ---
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="false")

    # --- 2. Platform & Localization Nodes ---
    
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
    
    # --- 3. Sensor & Middleware Launch Files ---
    
    # realsense D455
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']
        )
    )

    # Livox MID-360 (3D LiDAR)
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('livox_ros_driver2'), '/launch_ROS2/MID360_launch.py']
        )
    )

    # apriltag_ros
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('apriltag_ros'), '/launch/apriltag_detection.launch.py']
        )
    )
    
    # cygbot 2D lidar
    cyglidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('cyglidar_d2_ros2'), '/launch/cyglidar.launch.py']
        )
    )

    # RealSense 원본 토픽을 구독하여 Global Costmap용 새 토픽으로 재발행
    realsense_relay_node = Node(
        package='topic_tools', 
        executable='relay', 
        name='realsense_global_point_relay', 
        output='screen',
        parameters=[
            {"input_topic": "/camera1/depth/color/points"},
            {"output_topic": "/realsense/global_points"},
            {"type": "sensor_msgs/msg/PointCloud2"}
        ]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            # Launch Arguments
            use_sim_time,
            
            # Platform Nodes
            tetra_node,
            tetra_interface_node, 
            tetra_service_node,
            
            # Localization
            iahrs_driver_node,
            ekf_localization_node,
            rsp_node,
            
            # User Interface
            joy_node,
            
            # Middleware
            rosbridge_server,
            rosapi_node,
            
            # Sensor Launch Files (원본 토픽 발행)
            realsense_launch,
            livox_launch,
            cyglidar_launch,

            # Tag Detection (RealSense 토픽을 사용하므로 카메라 뒤에 배치)
            apriltag_launch, 
            
            # 릴레이 노드 (재발행)
            realsense_relay_node, 
        ]
    )
