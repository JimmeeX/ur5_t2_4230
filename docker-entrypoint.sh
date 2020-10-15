#!/bin/bash

# exit immediately if a command exits with a non-zero status (see `$ help set`)
set -e

# location of master
# export ROS_MASTER_URI=http://l:11311/
# echo "$ECHO_PREFIX" "set ROS master: " "$ROS_MASTER_URI"

# ROS installation
source /opt/ros/kinetic/setup.bash

# workspace holding custom ROS packages
source /root/catkin_ws/devel/setup.bash

exec "$@"