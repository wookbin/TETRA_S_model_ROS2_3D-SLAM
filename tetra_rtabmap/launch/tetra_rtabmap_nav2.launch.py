from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import time
import rclpy
from rclpy.node import Node as RclpyNode
from rtabmap_msgs.srv import PublishMap


def wait_and_publish_then_start_nav(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()

    rclpy.init()
    node = RclpyNode('publish_map_client')
    client = node.create_client(PublishMap, '/rtabmap/publish_map')

    max_wait_sec = 600  # max 10 minutes
    waited = 0
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /rtabmap/publish_map ...')
        waited += 1
        if waited >= max_wait_sec:
            node.get_logger().error('Timeout waiting for /rtabmap/publish_map')
            node.destroy_node()
            rclpy.shutdown()
            return []

    req = PublishMap.Request()
    req.global_map = True
    req.optimized = True
    req.graph_only = False

    future = client.call_async(req)
    end_by = time.monotonic() + 180.0
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > end_by:
            node.get_logger().error('PublishMap call timeout')
            node.destroy_node()
            rclpy.shutdown()
            return []

    if future.result() is None:
        node.get_logger().error('PublishMap call failed')
        node.destroy_node()
        rclpy.shutdown()
        return []

    node.get_logger().info('PublishMap OK')
    node.destroy_node()
    rclpy.shutdown()

    include_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tetra_navigation2'),
                'launch',
                'tetra_3d_navigation.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time_str}.items()
    )

    return [include_nav]


def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
    use_sim_time = (use_sim_time_str == 'true')
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    db_name = LaunchConfiguration('db_name').perform(context)
    db_path = f'/home/tetra/.ros/{db_name}.db'

    if not os.path.isfile(db_path):
        raise RuntimeError(f"Map file not found at {db_path}")

    rtabmap_arguments = ['--udb', db_path]
    rtabmap_arguments.extend([
        'Mem/NotLinkedNodesKept', 'false',
        'Mem/STMSize', '80',
        'Mem/LaserScanNormalK', '20',
        'Mem/IncrementalMemory', 'false',
        'Mem/InitWMWithAllNodes', 'true',
        'Optimizer/Strategy', '1',
        'Icp/VoxelSize', '0.1',
        'Icp/PointToPlaneK', '25',
        'Icp/PointToPlaneRadius', '0',
        'Icp/PointToPlane', 'true',
        'Icp/Iterations', '30',
        'Icp/Epsilon', '0.001',
        'Icp/MaxTranslation', '0.3',
        'Icp/MaxRotation', '0.3',
        'Icp/PointToPlaneMinComplexity', '0.01',
        'Icp/MaxCorrespondenceDistance', '0.28', #0.35
        'Icp/Strategy', '1',
        'Icp/OutlierRatio', '0.12', #'0.05'
        'Icp/CorrespondenceRatio', '0.12',
        'Icp/RangeMin', '0.1',
        'Icp/RangeMax', '25.0',
        'Vis/MaxFeatures', '0',
        'Vis/MinInliers', '0',
        'Kp/DetectorStrategy', '0',
        'Rtabmap/DetectionRate', '10.0',
        'RGBD/LinearUpdate', '0.08',
        'RGBD/AngularUpdate', '0.04',
        'RGBD/ProximityBySpace', 'true',
        'RGBD/ProximityByTime', 'false', 
        'RGBD/ProximityAngle', '180',
        'RGBD/ProximityMaxGraphDepth', '0',
        'RGBD/ProximityPathMaxNeighbors', '1',
        'RGBD/NeighborLinkRefining', 'true',
        'RGBD/LocalLoopDetectionSpace', 'true',
        'RGBD/LocalLoopDetectionTime', 'false', #true
        'RGBD/LocalLoopDetectionRadius', '6.0',
        'RGBD/StartAtOrigin', 'true',
        'RGBD/MaxOdomCacheSize', '5',
    ])


    rtabmap_util = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        output='screen',
        parameters=[{
            'assembling_time': 0.05,
            'fixed_frame_id': 'base_footprint',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
        ('cloud', lidar_topic),
        ]
    )
    
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'publish_tf': True,
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'frame_id': 'base_footprint',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False, #True
            'topic_queue_size': 100,
            'sync_queue_size': 100,
            'queue_size': 100,
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
            'Mem/BinDataKept': 'false',
            'Mem/ReduceGraph': 'true',
            'Grid/FromScan': 'true',
            'Grid/RayTracing': 'true',
            'Grid/3D': 'false',
            'Grid/RangeMax': '25.0',
            'Grid/RangeMin': '0.1',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MaxGroundHeight': '0.5',
            'Grid/NormalsSegmentation': 'false',
            'Grid/CellSize': '0.05',
            'database_path': db_path,
            'Reg/Force3DoF': 'true',
        }],
        arguments=rtabmap_arguments + ['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('scan_cloud', 'assembled_cloud'),
            ('odom', '/odometry/filtered'),
        ],
    )

    rviz_config_file = 'tetra_3d_navigation2.rviz'
    rviz_config_dir = os.path.join(
        get_package_share_directory('tetra_rtabmap'),
        'rviz',
        rviz_config_file
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    publish_then_nav = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=wait_and_publish_then_start_nav)]
    )

    return [rtabmap_util, rtabmap_slam, rviz_node, publish_then_nav]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('db_name', default_value='rtabmap', description='Name of the .db file without extension'),
        OpaqueFunction(function=launch_setup),
    ])
