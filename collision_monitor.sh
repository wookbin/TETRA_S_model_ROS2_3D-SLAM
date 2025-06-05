#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch tetra_navigation2 collision_monitor_node.launch.py
