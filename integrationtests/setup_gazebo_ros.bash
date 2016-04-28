#!/bin/bash
#
# Setup environment to make PX4 visible to ROS/Gazebo.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ "$#" -lt 1 ]
then
    echo usage: source setup_gazebo_ros.bash firmware_src_dir
    echo ""
    return 1
fi

SRC_DIR=$1

# setup Gazebo env and update package path
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SRC_DIR}/Tools/sitl_gazebo/models
export GAZEBO_PLUGIN_PATH=${SRC_DIR}/Tools/sitl_gazebo/Build/:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${SRC_DIR}/Tools/sitl_gazebo/Build/msgs/
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${SRC_DIR}
