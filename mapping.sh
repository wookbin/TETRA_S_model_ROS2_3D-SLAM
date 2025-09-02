#!/bin/bash
map_name=$1

export _colcon_cd_root=/opt/ros/jazzy/
source /usr/share/colcon_cd/function/colcon_cd.sh
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/tetra/cyclonedds.xml

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

#pkill -9 -f nav2_controller
#sleep 1

#ros2 launch tetra_cartographer cartographer.launch.py
#ros2 launch tetra_rtabmap tetra_rtabmap.launch.py mapping:=true db_name:=${map_name}
ros2 launch tetra_rtabmap tetra_rtabmap_mapping.launch.py db_name:=${map_name}
