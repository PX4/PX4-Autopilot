#!/bin/sh
#
# Fetch source repositories
#

# PX4 Firmware
git clone https://github.com/PX4/Firmware.git \
	&& cd Firmware \
	&& git checkout ros \
	&& cd ..

# euroc simulator
git clone https://github.com/PX4/euroc_simulator.git \
	&& cd euroc_simulator \
	&& git checkout px4_nodes \
	&& cd ..

# mav comm
git clone https://github.com/PX4/mav_comm.git

# glog catkin
git clone https://github.com/ethz-asl/glog_catkin.git

# catkin simple
git clone https://github.com/catkin/catkin_simple.git

echo "Execute catkin_make to compile all the sources."

