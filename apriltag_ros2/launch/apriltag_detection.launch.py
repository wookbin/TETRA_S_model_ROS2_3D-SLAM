from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[
                {'camera_frame': 'camera'}, #'camera1_depth_optical_frame'
                {'tag_family': 'tag36h11'},
                {'tag_size': 0.1}  # Adjust to your tag's size in meters
            ],
            remappings=[
                ('/image_rect', '/camera1/color/image_raw'), #'/camera/image_raw'
                ('/camera_info', '/camera1/color/camera_info'), #'/camera/camera_info'
            ],
        ),
    ])
