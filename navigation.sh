#!/bin/bash
map_name=$1
map_file="${map_name}.yaml"

export _colcon_cd_root=/opt/ros/jazzy/
source /usr/share/colcon_cd/function/colcon_cd.sh
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/tetra/cyclonedds.xml

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

while pgrep -f mapping.sh > /dev/null; do
 echo "wait for mapping die";
 sleep 1
done
# pkill -f cartographer.launch.py

#ros2 launch tetra_navigation2 tetra_navigation.launch.py map_name:=${map_file}
#ros2 launch tetra_rtabmap tetra_rtabmap.launch.py mapping:=false db_name:=${map_name}
ros2 launch tetra_rtabmap tetra_rtabmap_nav2.launch.py db_name:=${map_name}
sleep 2

