#!/bin/bash

cd Tools/sitl_gazebo/Build
cmake ..
make
sudo make install
 . /usr/share/gazebo/setup.sh
 . /usr/share/mavlink_sitl_gazebo/setup.sh
cd /home/marco/rover_src/Firmware
