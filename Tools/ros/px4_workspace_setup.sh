#!/bin/bash
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

# run this script from the root of your catkin_ws
source devel/setup.bash
cd src

# PX4 Firmware
git clone https://github.com/PX4/Firmware.git

# euroc simulator
git clone https://github.com/PX4/euroc_simulator.git

# mav comm
git clone https://github.com/PX4/mav_comm.git

# gflags catkin
git clone https://github.com/ethz-asl/gflags_catkin.git

# glog catkin
git clone https://github.com/ethz-asl/glog_catkin.git

# catkin simple
git clone https://github.com/catkin/catkin_simple.git

cd ..

catkin_make
