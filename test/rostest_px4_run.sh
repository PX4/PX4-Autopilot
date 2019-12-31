#! /bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
PX4_SRC_DIR=${DIR}/..

source /opt/ros/${ROS_DISTRO:-kinetic}/setup.bash
source ${PX4_SRC_DIR}/Tools/setup_gazebo.bash ${PX4_SRC_DIR} ${PX4_SRC_DIR}/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_SRC_DIR}:${PX4_SRC_DIR}/Tools/sitl_gazebo

export ROS_LOG_DIR="$HOME/.ros/ros_logs"
mkdir -p "$ROS_LOG_DIR"

rostest px4 "$@"
