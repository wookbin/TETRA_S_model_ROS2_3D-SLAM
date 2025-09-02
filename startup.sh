#!/bin/bash

export _colcon_cd_root=/opt/ros/jazzy/
source /usr/share/colcon_cd/function/colcon_cd.sh
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/tetra/cyclonedds.xml

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# which node

sleep 1

/home/tetra/.nvm/versions/node/v14.17.3/bin/pm2 stop all
cd /home/tetra/work/tetra-single-api
/home/tetra/.nvm/versions/node/v14.17.3/bin/pm2 start npm -- run start

#ros2 launch tetra_bringup tetra_bringup.launch.py
ros2 launch tetra_bringup tetra_3d_bringup.launch.py


