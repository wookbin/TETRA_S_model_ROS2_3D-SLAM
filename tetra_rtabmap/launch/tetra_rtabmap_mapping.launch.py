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
    
    incremental_memory = 'true'
    initwmwithallnodes = 'false'
    rviz_config_file = 'rtabmap_mapping.rviz'
    
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
        'Icp/VoxelSize', '0.05',
        'Icp/PointToPlaneK', '20',
        'Icp/PointToPlaneRadius', '0',
        'Icp/PointToPlane', 'true',
        'Icp/Iterations', '20',
        'Icp/Epsilon', '0.001',
        'Icp/MaxTranslation', '0.3',
        'Icp/MaxRotation', '0.3',
        'Icp/MaxCorrespondenceDistance', '0.3',
        'Icp/Strategy', '1',
        'Icp/OutlierRatio', '0.1',
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
            #'max_clouds': 10,
            'assembling_time': 0.1,
            'fixed_frame_id': 'base_footprint',
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
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
            'Mem/IncrementalMemory': incremental_memory,
            'Mem/InitWMWithAllNodes': initwmwithallnodes,
            'Mem/BinDataKept': 'false',
            'Mem/ReduceGraph': 'false',
            'Grid/FromScan': 'true',
            'Grid/RayTracing': 'true',
            'Grid/3D': 'true',
            'Grid/RangeMax': '20.0',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MaxGroundHeight': '0.5',
            'Grid/NormalsSegmentation': 'false',
            'Grid/CellSize': '0.05',
            'database_path': db_path,
            'Optimizer/Strategy': '1',
            'Reg/Force3DoF': 'true',
            
        }],
        remappings=[
            ('scan_cloud', 'assembled_cloud'), #lidar_topic
            ('odom', 'odom'),
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
