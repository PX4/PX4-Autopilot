#!/bin/sh
#
# Create workspace at current location and fetch source repositories
#

# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

WDIR=`pwd`
WORKSPACE=$WDIR/catkin_ws

# Setup workspace
mkdir -p $WORKSPACE/src
cd $WORKSPACE/src
catkin_init_workspace
cd $WORKSPACE
catkin_make
echo "source $WORKSPACE/devel/setup.bash" >> ~/.bashrc

# PX4 Firmware
cd $WORKSPACE/src
git clone https://github.com/PX4/Firmware.git

# euroc simulator
cd $WORKSPACE/src
git clone https://github.com/PX4/euroc_simulator.git \
	&& cd euroc_simulator \
	&& git checkout px4_nodes

# mav comm
cd $WORKSPACE/src
git clone https://github.com/PX4/mav_comm.git

# glog catkin
cd $WORKSPACE/src
git clone https://github.com/ethz-asl/glog_catkin.git

# catkin simple
cd $WORKSPACE/src
git clone https://github.com/catkin/catkin_simple.git

cd $WORKSPACE
catkin_make

