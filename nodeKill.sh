#!/bin/bash

export _colcon_cd_root=/opt/ros/jazzy/
source /usr/share/colcon_cd/function/colcon_cd.sh
export ROS_LOG_DIR=/tmp/ros_logs
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/tetra/cyclonedds.xml

pkill -f mapping.sh
pkill -f cartographer.launch.py
while pgrep -f mapping.sh > /dev/null; do
 echo "nodekill wait for mapping die";
 sleep 1
done
pkill -f cartographer_node
while pgrep -f cartographer_node > /dev/null; do
 echo "nodekill wait for cartographer_node die";
 sleep 1
done
pkill -f rviz2
while pgrep -f rviz2 > /dev/null; do
 echo "nodekill wait for rviz2 die";
 sleep 1
done
pkill -f cartographer_occupancy_grid_node
while pgrep -f cartographer_occupancy_grid_node > /dev/null; do
 echo "nodekill wait for cartographer_occupancy_grid_node die";
 sleep 1
done
pkill -f static_transform_publisher
while pgrep -f static_transform_publisher > /dev/null; do
 echo "nodekill wait for static_transform_publisher die";
 sleep 1
done

pkill -f rtabmap
while pgrep -f rtabmap > /dev/null; do
 echo "nodekill wait for rtabmap die";
 sleep 1
done


pkill -f navigation.sh
while pgrep -f navigation.sh > /dev/null; do
 echo "nodekill wait for navigation die";
 sleep 1
done
pkill -f amcl
while pgrep -f amcl > /dev/null; do
 echo "nodekill wait for amcl die";
 sleep 1
done
pkill -f lifecycle_manager
while pgrep -f lifecycle_manager > /dev/null; do
 echo "nodekill wait for lifecycle_manager die";
 sleep 1
done
pkill -f controller_server
while pgrep -f controller_server > /dev/null; do
 echo "nodekill wait for controller_server die";
 sleep 1
done
pkill -f controller_server
while pgrep -f controller_server > /dev/null; do
 echo "nodekill wait for controller_server die";
 sleep 1
done
pkill -f smoother_server
while pgrep -f smoother_server > /dev/null; do
 echo "nodekill wait for smoother_server die";
 sleep 1
done
pkill -f planner_server
while pgrep -f planner_server > /dev/null; do
 echo "nodekill wait for planner_server die";
 sleep 1
done
pkill -f behavior_server
while pgrep -f behavior_server > /dev/null; do
 echo "nodekill wait for behavior_server die";
 sleep 1
done
pkill -f velocity_smoother
while pgrep -f velocity_smoother > /dev/null; do
 echo "nodekill wait for velocity_smoother die";
 sleep 1
done
pkill -f collision_monitor
while pgrep -f collision_monitor > /dev/null; do
 echo "nodekill wait for collision_monitor die";
 sleep 1
done
pkill -f bt_navigator
while pgrep -f bt_navigator > /dev/null; do
 echo "nodekill wait for bt_navigator die";
 sleep 1
done
pkill -f waypoint_follower
while pgrep -f waypoint_follower > /dev/null; do
 echo "nodekill wait for waypoint_follower die";
 sleep 1
done
pkill -f opennav_docking
while pgrep -f opennav_docking > /dev/null; do
 echo "nodekill wait for opennav_docking die";
 sleep 1
done
pkill -f virtual_wall_node
while pgrep -f virtual_wall_node > /dev/null; do
 echo "nodekill wait for virtual_wall_node die";
 sleep 1
done

sleep 1
