#!/bin/sh
# run this script from the root of your catkin_ws
source devel/setup.bash
cd src

# PX4 Firmware
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout ros
cd ..

# euroc simulator
git clone https://github.com/PX4/euroc_simulator.git
cd euroc_simulator
git checkout px4_nodes
cd ..

# mav comm
git clone https://github.com/PX4/mav_comm.git

# glog catkin
git clone https://github.com/ethz-asl/glog_catkin.git

# catkin simple
git clone https://github.com/catkin/catkin_simple.git

# drcsim (for scenery and models)
hg clone https://bitbucket.org/osrf/osrf-common
hg clone https://bitbucket.org/osrf/sandia-hand
hg clone https://bitbucket.org/osrf/drcsim

cd ..

catkin_make
