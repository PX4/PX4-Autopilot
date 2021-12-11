# !/bin/bash

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$1/Tools/sitl_ign_gazebo/build
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$1/Tools/sitl_ign_gazebo/models
