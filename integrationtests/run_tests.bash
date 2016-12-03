#!/bin/bash
#
# Starts tests from within the container
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository
set -e

# TODO move to docker image
pip install px4tools pymavlink -q

# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

# Initialize our own variables:
do_clean=true
gui=false

while getopts "h?og" opt; do
    case "$opt" in
    h|\?)
		echo """
		$0 [-h] [-o] [-g]
		-h show help
		-o don't clean before building (to save time)
		-g run gazebo gui
		"""
        exit 0
        ;;
    o)  do_clean=false
		echo "not cleaning"
        ;;
    g)  gui=true
        ;;
    esac
done


# determine the directory of the source given the directory of this script
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null
ORIG_SRC=$(dirname $SCRIPTPATH)

# set paths
JOB_DIR=$(dirname $ORIG_SRC)
CATKIN_DIR=$JOB_DIR/catkin
BUILD_DIR=$CATKIN_DIR/build/px4
SRC_DIR=${CATKIN_DIR}/src/px4

echo setting up ROS paths
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
export ROS_HOME=$JOB_DIR/.ros
export ROS_LOG_DIR=$ROS_HOME/log
export ROS_TEST_RESULT_DIR=$ROS_HOME/test_results/px4
export PX4_LOG_DIR=$ROS_HOME/rootfs/fs/microsd/log
TEST_RESULT_TARGET_DIR=$JOB_DIR/test_results

# TODO
# BAGS=$ROS_HOME
# CHARTS=$ROS_HOME/charts
# EXPORT_CHARTS=/sitl/testing/export_charts.py

if $do_clean
then
	echo cleaning
	rm -rf $CATKIN_DIR
	rm -rf $ROS_HOME
	rm -rf $TEST_RESULT_TARGET_DIR
else
	echo skipping clean step
fi

echo "=====> compile ($SRC_DIR)"
mkdir -p $ROS_HOME
mkdir -p $CATKIN_DIR/src
mkdir -p $TEST_RESULT_TARGET_DIR
if ! [ -d $SRC_DIR ]
then
	ln -s $ORIG_SRC $SRC_DIR
	ln -s $ORIG_SRC/Tools/sitl_gazebo ${CATKIN_DIR}/src/mavlink_sitl_gazebo
fi
cd $CATKIN_DIR
catkin_make
. ./devel/setup.bash
echo "<====="

# print paths to user
echo -e "JOB_DIR\t\t: $JOB_DIR"
echo -e "ROS_HOME\t: $ROS_HOME"
echo -e "CATKIN_DIR\t: $CATKIN_DIR"
echo -e "BUILD_DIR\t: $BUILD_DIR"
echo -e "SRC_DIR\t\t: $SRC_DIR"
echo -e "ROS_TEST_RESULT_DIR\t: $ROS_TEST_RESULT_DIR"
echo -e "ROS_LOG_DIR\t\t: $ROS_LOG_DIR"
echo -e "PX4_LOG_DIR\t\t: $PX4_LOG_DIR"
echo -e "TEST_RESULT_TARGET_DIR\t: $TEST_RESULT_TARGET_DIR"

# don't exit on error anymore (because single tests or exports might fail)
# however, stop executing tests after the first failure
set +e
echo "=====> run tests"
test $? -eq 0 && rostest px4 mavros_posix_tests_iris.launch gui:=$gui

# commented out optical flow test for now since ci server has
# an issue producing the simulated flow camera currently
#test $? -eq 0 && rostest px4 mavros_posix_tests_iris_opt_flow.launch gui:=$gui

test $? -eq 0 && rostest px4 mavros_posix_tests_standard_vtol.launch gui:=$gui
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
cp -r $ROS_TEST_RESULT_DIR/* ${TEST_RESULT_TARGET_DIR}
cp -r $ROS_LOG_DIR/* ${TEST_RESULT_TARGET_DIR}
cp -r $PX4_LOG_DIR/* ${TEST_RESULT_TARGET_DIR}
# cp $BAGS/*.bag ${TEST_RESULT_TARGET_DIR}/
# cp -r $CHARTS ${TEST_RESULT_TARGET_DIR}/
echo "<====="

# need to return error if tests failed, else Jenkins won't notice the failure
exit $TEST_RESULT
