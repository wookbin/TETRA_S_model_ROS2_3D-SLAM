from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import rclpy
from rclpy.node import Node as RclpyNode
from rtabmap_msgs.srv import PublishMap


def call_publish_map_service(context):
    rclpy.init()
    node = RclpyNode('publish_map_client')
    client = node.create_client(PublishMap, '/rtabmap/publish_map')

    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Service /rtabmap/publish_map not available')
        rclpy.shutdown()
        return []

    request = PublishMap.Request()
    request.global_map = True
    request.optimized = True  #False
    request.graph_only = False

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None:
        node.get_logger().info('PublishMap service called successfully')
    else:
        node.get_logger().error('Failed to call PublishMap service')

    node.destroy_node()
    rclpy.shutdown()
    return []

def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
    use_sim_time = (use_sim_time_str == 'true')
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    db_name = LaunchConfiguration('db_name').perform(context)
    db_path = f'/home/tetra/.ros/{db_name}.db'

    incremental_memory = 'false'
    initwmwithallnodes = 'true'
    rviz_config_file = 'tetra_3d_navigation2.rviz'
    
    if not os.path.isfile(db_path):
        raise RuntimeError(f"Map file not found at {db_path}")

    rviz_config_dir = os.path.join(
        get_package_share_directory('tetra_rtabmap'),
        'rviz',
        rviz_config_file
    )

    rtabmap_arguments = ['--udb', db_path]

    rtabmap_arguments.extend([
        'Mem/NotLinkedNodesKept', 'false',
        'Mem/STMSize', '30',
        'Mem/LaserScanNormalK', '20',
        'Optimizer/Strategy','1',
        'Icp/VoxelSize', '0.1', # 0.15
        'Icp/PointToPlaneK', '15', #20
        'Icp/PointToPlaneRadius', '0',
        'Icp/PointToPlane', 'true',
        'Icp/Iterations', '10',
        'Icp/Epsilon', '0.001',
        'Icp/MaxTranslation', '0.5',
        'Icp/MaxRotation', '0.5',
        'Icp/MaxCorrespondenceDistance', '0.5', #1.0
        'Icp/Strategy', '1',
        'Icp/OutlierRatio', '0.6', #0.7
        'Icp/CorrespondenceRatio', '0.2',
        'Rtabmap/DetectionRate', '10.0',
        'Vis/MaxFeatures', '0',
        'Vis/MinInliers', '0',
    ])

    rtabmap_util = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        output='screen',
        parameters=[{
            'max_clouds': 20,
            'assembling_time': 0.1,
            'fixed_frame_id': 'map',
            'use_sim_time': use_sim_time,
        }]
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'use_sim_time': use_sim_time,
            'Mem/IncrementalMemory': incremental_memory,
            'Mem/InitWMWithAllNodes': initwmwithallnodes,
            'Mem/BinDataKept': 'false', # add... default: false
            'Mem/ReduceGraph': 'true', # add... default: false
            'Grid/FromScan': 'true',
            'Grid/RayTracing': 'true',
            'Grid/3D': 'true',
            'Grid/RangeMax': '15.0',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MaxGroundHeight': '0.5', #0.05
            'Grid/NormalsSegmentation': 'false',
            'Grid/CellSize': '0.05',
            'database_path': db_path,
            'Optimizer/Strategy': '1',
            'Reg/Force3DoF': 'true', #add.. only 2D pose
            
        }],
        #arguments=rtabmap_arguments,
        arguments=rtabmap_arguments + ['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('scan_cloud', lidar_topic),
            ('odom', 'odom'),
        ],
    )

    rtabmap_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # timer action...
    publish_map_and_nav_timer = TimerAction(
        period=5.0,
        actions=[
            OpaqueFunction(function=call_publish_map_service),
            rtabmap_rviz2,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('tetra_navigation2'),
                        'launch',
                        'tetra_3d_navigation.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time_str
                }.items()
            )
        ]
    )

    return [
        rtabmap_util,
        rtabmap_slam,
        publish_map_and_nav_timer,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('db_name', default_value='rtabmap', description='Name of the .db file without extension'),
        OpaqueFunction(function=launch_setup),
    ])


