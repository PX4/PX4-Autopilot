#!/bin/bash
#
# Starts tests from within the container
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository
set -e

if [ "$#" -lt 1 ]
then
	echo usage: run_tests.bash firmware_src_dir
	echo ""
	exit 1
fi

SRC_DIR=$1
JOB_DIR=$SRC_DIR/..
BUILD=posix_sitl_default
# TODO
ROS_TEST_RESULT_DIR=/root/.ros/test_results/px4
ROS_LOG_DIR=/root/.ros/log
PX4_LOG_DIR=${SRC_DIR}/build_${BUILD}/src/firmware/posix/rootfs/fs/microsd/log
TEST_RESULT_TARGET_DIR=$JOB_DIR/test_results
# BAGS=/root/.ros
# CHARTS=/root/.ros/charts
# EXPORT_CHARTS=/sitl/testing/export_charts.py

# source ROS env
if [ -f /opt/ros/indigo/setup.bash ]
then
	source /opt/ros/indigo/setup.bash
elif [ -f /opt/ros/kinetic/setup.bash ]
then
	source /opt/ros/kinetic/setup.bash
else
	echo "could not find /opt/ros/{ros-distro}/setup.bash"
	exit 1
fi
source $SRC_DIR/integrationtests/setup_gazebo_ros.bash $SRC_DIR

echo "deleting previous test results ($TEST_RESULT_TARGET_DIR)"
if [ -d ${TEST_RESULT_TARGET_DIR} ]; then
	rm -r ${TEST_RESULT_TARGET_DIR}
fi

# FIXME: Firmware compilation seems to CD into this directory (/root/Firmware)
# when run from "run_container.bash". Why?
if [ -d /root/Firmware ]; then
	rm /root/Firmware
fi
ln -s ${SRC_DIR} /root/Firmware

echo "=====> compile ($SRC_DIR)"
cd $SRC_DIR
make ${BUILD}
make --no-print-directory gazebo_build
echo "<====="

# don't exit on error anymore from here on (because single tests or exports might fail)
set +e
echo "=====> run tests"
rostest px4 mavros_posix_tests_iris.launch
rostest px4 mavros_posix_tests_standard_vtol.launch
TEST_RESULT=$?
echo "<====="

# TODO
echo "=====> process test results"
# cd $BAGS
# for bag in `ls *.bag`
# do
# 	echo "processing bag: $bag"
# 	python $EXPORT_CHARTS $CHARTS $bag
# done

echo "copy build test results to job directory"
mkdir -p ${TEST_RESULT_TARGET_DIR}
cp -r $ROS_TEST_RESULT_DIR/* ${TEST_RESULT_TARGET_DIR}
cp -r $ROS_LOG_DIR/* ${TEST_RESULT_TARGET_DIR}
cp -r $PX4_LOG_DIR/* ${TEST_RESULT_TARGET_DIR}
# cp $BAGS/*.bag ${TEST_RESULT_TARGET_DIR}/
# cp -r $CHARTS ${TEST_RESULT_TARGET_DIR}/
echo "<====="

# need to return error if tests failed, else Jenkins won't notice the failure
exit $TEST_RESULT
