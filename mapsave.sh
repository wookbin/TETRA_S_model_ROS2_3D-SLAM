#!/bin/bash
map_name=$1

export _colcon_cd_root=/opt/ros/jazzy/
source /usr/share/colcon_cd/function/colcon_cd.sh
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/tetra/cyclonedds.xml

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd /home/tetra/ros2_ws/src/tetra_navigation2/maps/

ros2 run nav2_map_server map_saver_cli -f ${map_name}
#ros2 run nav2_map_server map_saver_cli -f ${map_name} --ros-args -p save_map_timeout:=10000

