#!/bin/bash
#
# Setup environment to make PX4 visible to Gazebo.
#
# Note, this is not necessary if using a ROS catkin workspace with the px4
# package as the paths are exported.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ "$#" != 2 ]
then
    echo usage: source setup_gazebo.bash src_dir build_dir
    echo ""
    return 1
fi

SRC_DIR=$1
BUILD_DIR=$2

# setup Gazebo env and update package path
export GAZEBO_PLUGIN_PATH=${BUILD_DIR}/build_gazebo:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SRC_DIR}/Tools/sitl_gazebo/models
# Disabling the remote model download seems only necessary with Gazebo 6
#export GAZEBO_MODEL_DATABASE_URI=""
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${SRC_DIR}/Tools/sitl_gazebo/Build/msgs/:${BUILD_DIR}/build_gazebo
echo -e "GAZEBO_PLUGIN_PATH $GAZEBO_PLUGIN_PATH"
echo -e "GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH"
echo -e "LD_LIBRARY_PATH $LD_LIBRARY_PATH"
