from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    mapping = LaunchConfiguration('mapping').perform(context).lower()
    db_path = LaunchConfiguration('map_path').perform(context)
    use_sim_time = (use_sim_time_str == 'true')
    incremental_memory = 'true' if mapping == 'true' else 'false'
    initwmwithallnodes = 'false' if mapping == 'true' else 'true'
    
    if mapping == 'true':
        rviz_config_file = 'rtabmap_mapping.rviz'
    else:
        rviz_config_file = 'tetra_3d_navigation2.rviz'
        rviz_config_dir = os.path.join(get_package_share_directory('tetra_rtabmap'),
        'rviz',
        rviz_config_file)

    

    rtabmap_arguments = []
    if mapping == 'false':
        if not os.path.isfile(db_path):
            raise RuntimeError(f"map file not found at {db_path}")
        rtabmap_arguments.extend(['--udb', db_path])

    rtabmap_arguments.extend([
        'Mem/NotLinkedNodesKept', 'false',
        'Mem/STMSize', '30',
        'Mem/LaserScanNormalK', '20',
        'Reg/Strategy', '1',
        'Icp/VoxelSize', '0.5',
        'Icp/PointToPlaneK', '20',
        'Icp/PointToPlaneRadius', '0',
        'Icp/PointToPlane', 'true',
        'Icp/Iterations', '10',
        'Icp/Epsilon', '0.001',
        'Icp/MaxTranslation', '3',
        'Icp/MaxCorrespondenceDistance', '1',
        'Icp/Strategy', '1',
        'Icp/OutlierRatio', '0.7',
        'Icp/CorrespondenceRatio', '0.2',
        'Rtabmap/DetectionRate', '10.0', # 10Hz...

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
        }],
        remappings=[
            #('cloud', '/rtabmap/cloud'),  # default topic for cloud input from RTAB-Map
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
            'wait_for_transform': 0.2,
            'use_sim_time': use_sim_time,
            'Mem/IncrementalMemory': incremental_memory,
            'Mem/InitWMWithAllNodes': initwmwithallnodes,
            'Grid/FromScan': 'true',
            'Grid/RayTracing':'true', # Fill empty space
            'Grid/3D': 'true', #'false',
            'Grid/RangeMax': '15.0',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MaxGroundHeight': '0.05', #'0.0',
            'Grid/NormalsSegmentation': 'false',
            'Grid/CellSize': '0.05',

        }],
        arguments=rtabmap_arguments,
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

    return [
        rtabmap_util,
        rtabmap_slam,
        rtabmap_rviz2,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('mapping', default_value='true', description='true: mapping, false: localization'),
        DeclareLaunchArgument('map_path', default_value='/home/tetra/.ros/rtabmap.db', description='map_path'),
        OpaqueFunction(function=launch_setup),
    ])

