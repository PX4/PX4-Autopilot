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
verbose=false

while getopts "h?ogv" opt; do
    case "$opt" in
    h|\?)
		echo """
		$0 [-h] [-o] [-g] [-v]
		-h show help
		-o don't clean before building (to save time)
		-g run gazebo gui
		-v verbose output
		"""
        exit 0
        ;;
    o)  do_clean=false
		echo "not cleaning"
        ;;
    g)  gui=true
        ;;
    v)  verbose=true
		;;
    esac
done

if $verbose 
	then
	echo "run_tests.bash called."
fi


# determine the directory of the source given the directory of this script
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null
ORIG_SRC=$(dirname $SCRIPTPATH)

# =============================================================================================== #
# =================================== Install/build OpticalFlow ================================= #
# =============================================================================================== #
echo "=====> Building/installing OpticalFlow"

# OpticalFlow is not a catkin package, so we just use a standard CMake build/install
# procedure for it, before passing a modified CMAKE_MODULE_PATH to catkin

OPTICAL_FLOW_BUILD_DIR=$HOME/OpticalFlow_build/
OPTICAL_FLOW_INSTALL_DIR=$HOME/OpticalFlow_install/

if $do_clean
then
	echo "Deleting OpticalFlow build and install directories..."
	rm -rf $OPTICAL_FLOW_BUILD_DIR
	rm -rf $OPTICAL_FLOW_INSTALL_DIR
fi

# Create new directory for OpticalFlow build output
mkdir -p $OPTICAL_FLOW_BUILD_DIR
cd $OPTICAL_FLOW_BUILD_DIR

# Generate Makefiles and FindOpticalFlow.cmake, specifying a custom install directory
# (so it doesn't pollute the system space and install to /usr/local/)
cmake $ORIG_SRC/Tools/OpticalFlow -DCMAKE_INSTALL_PREFIX=$OPTICAL_FLOW_INSTALL_DIR

# Build OpticalFlow
make

# Install OpticalFlow
make install

# debug
#exit 0

# =============================================================================================== #
# ======================================== ROS/catkin Setup ===================================== #
# =============================================================================================== #
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
	# Symbolic links to catkin packages below. Note that submodules like
	# rotors_simulator and mav_comm contain many catkin packages.
	# Symbolic link for the rotors_simulation sub-module
	#ln -s $ORIG_SRC/Tools/sitl_gazebo ${CATKIN_DIR}/src/mavlink_sitl_gazebo
	ln -s $ORIG_SRC/Tools/rotors_simulator/rotors_gazebo ${CATKIN_DIR}/src/rotors_gazebo
	ln -s $ORIG_SRC/Tools/rotors_simulator/rotors_gazebo_plugins ${CATKIN_DIR}/src/rotors_gazebo_plugins
	ln -s $ORIG_SRC/Tools/mav_comm/mav_msgs ${CATKIN_DIR}/src/mav_msgs
	
fi
cd $CATKIN_DIR

# These build parameters are used by the CMakeLists.txt in the
# rotors_gazebo_plugins and rotors_gazebo packages.
# ADDITIONAL_INCLUDE_DIRS=...				This assumes mav_msgs has been symlinked to the catkin workspace as per above
# BUILD_MAVLINK_INTERFACE_PLUGIN=TRUE		Build the MAVLink interface plugin for Gazebo
# BUILD_OCTOMAP_PLUGIN=FALSE				Do not build the Octomap plugin for Gazebo
# BUILD_OPTICAL_FLOW_PLUGIN=TRUE			Build the optical flow plugin for Gazebo
# NO_ROS=TRUE 								Instruct the Gazebo plugins to build without ROS dependencies. Note that even though ROS is used for these tests, PX4 uses the Gazebo plugins without any ROS dependencies.
# CMAKE_MODULE_PATH=$OPTICAL_FLOW_INSTALL_DIR		Tell catkin where to find the OpticalFlow installation, so find(OpticalFlow) works.
catkin_make \
	-DADDITIONAL_INCLUDE_DIRS=$CATKIN_DIR/src/mav_msgs/include/	\
	-DBUILD_MAVLINK_INTERFACE_PLUGIN=TRUE \
	-DMAVLINK_HEADER_DIR=$ORIG_SRC/mavlink/include/mavlink/v1.0/ \
	-DBUILD_OCTOMAP_PLUGIN=FALSE \
	-DBUILD_OPTICAL_FLOW_PLUGIN=TRUE -DNO_ROS=TRUE \
	-DCMAKE_MODULE_PATH=$OPTICAL_FLOW_INSTALL_DIR

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
# --text can be added to display more debug info to the rostest command, although it
#			affects performance and should not be enabled in production build!
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
