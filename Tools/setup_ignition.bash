#!/bin/bash
#
# Setup environment to make PX4 visible to Gazebo.
#
# Note, this is not necessary if using a ROS catkin workspace with the px4
# package as the paths are exported.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ "$#" != 2 ]; then
    echo -e "usage: source setup_gazebo.bash src_dir build_dir\n"
    return 1
fi

SRC_DIR=$1
BUILD_DIR=$2

# setup Gazebo env and update package path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${SRC_DIR}/build/px4_sitl_default/build_ign_gazebo
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:${SRC_DIR}/build/px4_sitl_default/build_ign_gazebo
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:${SRC_DIR}/Tools/simulation-ignition/models

echo -e "LD_LIBRARY_PATH $LD_LIBRARY_PATH"
