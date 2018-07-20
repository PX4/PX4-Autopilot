#! /bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
PX4_SRC_DIR=${DIR}/..

source /opt/ros/kinetic/setup.bash
mkdir -p ${PX4_SRC_DIR}/catkin_ws/src
cd ${PX4_SRC_DIR}/catkin_ws/
git clone --depth=1 https://github.com/PX4/avoidance.git src/

catkin init
catkin build local_planner --cmake-args -DCMAKE_BUILD_TYPE=Release

source ${PX4_SRC_DIR}/catkin_ws/devel/setup.bash
export CATKIN_SETUP_UTIL_ARGS=--extend

source $DIR/rostest_px4_run.sh "$@"

