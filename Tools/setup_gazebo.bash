#!/bin/bash
#
# Setup environment to make PX4 visible to Gazebo.
#
# Note, this is not necessary if using a ROS catkin workspace with the px4
# package as the paths are exported.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

echo "setup_gazebo.bash called."

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

# Set model directory to folder within rotors_simulator sub-module
# (contains .model and .sdf files)
MODEL_DIRECTORY=${SRC_DIR}/Tools/rotors_simulator/rotors_gazebo/models

if [ ! -d "$MODEL_DIRECTORY" ]; then
   echo "ERROR: The MODEL_DIRECTORY '${MODEL_DIRECTORY}' was not found."
   # Return error
   return 1
fi
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${MODEL_DIRECTORY}

# Disabling the remote model download seems only necessary with Gazebo 6
#export GAZEBO_MODEL_DATABASE_URI=""

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${BUILD_DIR}/build_gazebo
echo -e "GAZEBO_PLUGIN_PATH $GAZEBO_PLUGIN_PATH"
echo -e "GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH"
echo -e "LD_LIBRARY_PATH $LD_LIBRARY_PATH"
