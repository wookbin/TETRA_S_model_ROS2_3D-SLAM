from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
    use_sim_time = (use_sim_time_str == 'true')
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    db_name = LaunchConfiguration('db_name').perform(context)
    db_path = f'/home/tetra/.ros/{db_name}.db'
    rviz_config_file = 'rtabmap_mapping.rviz'
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('tetra_rtabmap'),
        'rviz',
        rviz_config_file
    )

    rtabmap_arguments = ['--udb', db_path]

    rtabmap_arguments.extend([
        'Mem/NotLinkedNodesKept', 'false',
        'Mem/STMSize', '80',
        'Mem/LaserScanNormalK', '20',
        'Mem/IncrementalMemory', 'true',
        'Mem/InitWMWithAllNodes', 'false',
        'Optimizer/Strategy','1',
        'Icp/VoxelSize', '0.1',
        'Icp/PointToPlaneK', '25',
        'Icp/PointToPlaneRadius', '0',
        'Icp/PointToPlane', 'true',
        'Icp/Iterations', '30',
        'Icp/Epsilon', '0.001',
        'Icp/MaxTranslation', '0.3',
        'Icp/MaxRotation', '0.3',
        'Icp/MaxCorrespondenceDistance', '0.28',
        'Icp/Strategy', '1',
        'Icp/PointToPlaneMinComplexity', '0.01', #add...
        'Icp/OutlierRatio', '0.12', #0.1
        'Icp/CorrespondenceRatio', '0.12',
        'Icp/RangeMin', '0.05',
        'Icp/RangeMax', '25.0',
        'Vis/MaxFeatures', '0',
        'Vis/MinInliers', '0',
        'Kp/DetectorStrategy', '0',
        'Rtabmap/DetectionRate', '5.0',
        'RGBD/NeighborLinkRefining', 'true',
        'RGBD/ProximityBySpace', 'true', 
        'RGBD/ProximityByTime', 'false', 
        'RGBD/ProximityAngle', '180',
        'RGBD/ProximityMaxGraphDepth', '0',
        'RGBD/ProximityPathMaxNeighbors', '5', #'3'
        'RGBD/LinearUpdate', '0.08',
        'RGBD/AngularUpdate', '0.04',
        'RGBD/CreateIntermediateNodes', 'true',
        'RGBD/LocalLoopDetectionSpace', 'true',
        'RGBD/LocalLoopDetectionTime', 'true',
        'RGBD/LocalLoopDetectionRadius', '6.0', #5.0
        'RGBD/StartAtOrigin', 'true', # mapping mode: false || localization mode: true
        'RGBD/OptimizeFromGraphEnd', 'false',
        'RGBD/OptimizeMaxError', '4',
        'RGBD/MaxOdomCacheSize', '10',
    ])

    rtabmap_util = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        output='screen',
        parameters=[{
            'assembling_time': 0.08,
            'fixed_frame_id': 'odom', #'base_footprint',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
        ('cloud', lidar_topic),
        # ('assembled_cloud', 'assembled_cloud')
        ]
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
            'topic_queue_size': 100,
            'sync_queue_size': 100,
            'queue_size': 100,
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
            'Mem/BinDataKept': 'false',
            'Mem/ReduceGraph': 'false',
            'Grid/FromScan': 'true',
            'Grid/RayTracing': 'true',
            'Grid/3D': 'true',
            'Grid/RangeMax': '25.0',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MaxGroundHeight': '0.5',
            'Grid/NormalsSegmentation': 'false',
            'Grid/CellSize': '0.05',
            'database_path': db_path,
            'Reg/Force3DoF': 'true', # 2D SLAM
            
        }],
        remappings=[
            ('scan_cloud', 'assembled_cloud'), #lidar_topic
            ('odom', '/odometry/filtered'),
        ],
        arguments=rtabmap_arguments + ['--ros-args', '--log-level', 'WARN'],
    )

    rtabmap_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    actions = [
        rtabmap_util,
        rtabmap_slam,
        rtabmap_rviz2,
    ]
    
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('db_name', default_value='rtabmap', description='Name of the .db file without extension'),
        OpaqueFunction(function=launch_setup),
    ])
