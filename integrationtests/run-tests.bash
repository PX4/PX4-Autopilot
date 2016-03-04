#!/bin/bash
#
# Starts tests from within the container
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository
set -e

SRC_DIR=/root/Firmware
# TODO
# TEST_RESULTS=/root/.ros/px4/test_results
# BAGS=/root/.ros
# CHARTS=/root/.ros/charts
# EXPORT_CHARTS=/sitl/testing/export_charts.py

# source ROS env, setup Gazebo env
source /opt/ros/indigo/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SRC_DIR}/Tools/sitl_gazebo/models
export GAZEBO_PLUGIN_PATH=${SRC_DIR}/Tools/sitl_gazebo/Build/:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${SRC_DIR}/Tools/sitl_gazebo/Build/msgs/
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${SRC_DIR}

echo "deleting previous test results"
if [ -d /job/test_results ]; then
	rm -r /job/test_results
fi

echo "linking source to test"
if [ -d "${SRC_DIR}" ]; then
	rm -r ${SRC_DIR}
fi
ln -s /job/Firmware ${SRC_DIR}

echo "=====> compile"
cd $SRC_DIR
make posix_sitl_default
mkdir -p Tools/sitl_gazebo/Build
cd Tools/sitl_gazebo/Build
cmake -Wno-dev ..
make -j4
echo "<====="

# don't exit on error anymore from here on (because single tests or exports might fail)
set +e
echo "=====> run tests"
rostest px4 mavros_tests_posix.launch
TEST_RESULT=$?
echo "<====="

# TODO
# echo "=====> process test results"
# cd $BAGS
# for bag in `ls *.bag`
# do
# 	echo "processing bag: $bag"
# 	python $EXPORT_CHARTS $CHARTS $bag
# done

# echo "copy build test results to job directory"
# cp -r $TEST_RESULTS /job/
# cp $BAGS/*.bag /job/test_results/
# cp -r $CHARTS /job/test_results/
# echo "<====="

# need to return error if tests failed, else Jenkins won't notice the failure
exit $TEST_RESULT
