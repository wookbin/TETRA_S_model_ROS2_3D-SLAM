# TETRA_S_model_ROS2_3D-SLAM
TETRA_S_Model_ROS2_jazzy_3D SLAM &amp; NAV2

ros2-jazzy version package

[중요] realsense의 pointcloud.enable와 enable_color prameter는 다음과 같이 설정.
* 도킹시에는 pointcloud.enable = false | enable_color = true
* 주행시에는 pointcloud.enable = true | enable_color = false

<br />

사용되는 센서:
   SICK Tim571(2D LiDAR), Cygbot D2(2D/3D LiDAR), Realsense D435(3D Depth Camera), IAHRS(IMU), Livox_MID-360(3D LiDAR)

- SICK Tim571: https://github.com/SICKAG/sick_scan_xd
- Cygbot D2: https://github.com/CygLiDAR-ROS/cyglidar_d2
- Realsense D435F: https://github.com/IntelRealSense/realsense-ros
- IAHRS : https://github.com/wookbin/iahrs_driver_ros2
- Livox MID-360: https://github.com/Livox-SDK/livox_ros_driver2

<br />

사용하는 패키지:
- apriltag_ros: https://github.com/AprilRobotics/apriltag_ros
- rtabmap(3D SLAM): https://github.com/introlab/rtabmap_ros
- nav2(Navigation): https://github.com/ros-navigation/navigation2
- laser_filters(scan filter): https://github.com/ros-perception/laser_filters
- topic_tools: https://github.com/ros-tooling/topic_tools/tree/jazzy
  
<br />



예시에 대한 설명 링크: 
- rtabmap을 이용한 3D SLAM_mapping mode: https://blog.naver.com/zzang0736/223881606310
- rtabmap을 이용한 3D SLAM_localization mode &amp; NAV2: https://blog.naver.com/zzang0736/223888350662
