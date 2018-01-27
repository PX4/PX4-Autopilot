#!/bin/bash

source /usr/share/gazebo/setup.sh

GAZEBO_MASTER_IP=127.0.0.1
GAZEBO=gazebo

#in virtual machine or on separate network host
#GAZEBO_MASTER_IP=EXT.ERN.AL.IP
#GAZEBO=gzserver

gazebo_world=$1
shift

for g;do
  GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$g/build
  GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$g/models
done

# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH

# Disable online model lookup since this is quite experimental and unstable
#export GAZEBO_MODEL_DATABASE_URI=""

export GAZEBO_MASTER_IP
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
export GAZEBO_IP=$GAZEBO_MASTER_IP

$GAZEBO --verbose $gazebo_world
