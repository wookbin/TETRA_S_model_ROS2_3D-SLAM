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
        'Mem/STMSize', '30',
        'Mem/LaserScanNormalK', '20',
        'Mem/IncrementalMemory', 'true',
        'Mem/InitWMWithAllNodes', 'false',
        'Optimizer/Strategy','1',
        'Icp/VoxelSize', '0.05',
        'Icp/PointToPlaneK', '20',
        'Icp/PointToPlaneRadius', '0',
        'Icp/PointToPlane', 'true',
        'Icp/Iterations', '30',
        'Icp/Epsilon', '0.005',
        'Icp/MaxTranslation', '0.3',
        'Icp/MaxRotation', '0.3',
        'Icp/MaxCorrespondenceDistance', '0.3',
        'Icp/Strategy', '1',
        'Icp/OutlierRatio', '0.1',
        'Icp/CorrespondenceRatio', '0.1',
        'Vis/MaxFeatures', '1000', #0
        'Vis/MinInliers', '15', #0
        'Rtabmap/DetectionRate', '1.0', #10.0
        'RGBD/LinearUpdate', '0.10',        # 10cm 이상 이동해야 새 키프레임
        'RGBD/AngularUpdate', '0.05',       # 2.86° 이상 회전해야 새 키프레임
        'RGBD/CreateIntermediateNodes', 'false',
        'RGBD/LocalLoopDetectionSpace', 'true',
        'RGBD/LocalLoopDetectionTime', 'true',
        'RGBD/LocalLoopDetectionRadius', '3.0',   # 반경 3m에서 loop 후보 찾기
        'RGBD/StartAtOrigin', 'false', # mapping mode: false || localization mode: true
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
            'topic_queue_size': 30,         # 동기화 유연성
            'sync_queue_size': 30,          # 동기화 유연성
            'queue_size': 30,               # 동기화 유연성
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
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
            'Reg/Force3DoF': 'true',
            
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
