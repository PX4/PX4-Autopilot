#!/bin/bash
echo "1"
cd Tools/sitl_gazebo
echo "2"
git submodule update --init --recursive
echo "3"
mkdir -p Build
echo "4"
cd Build
echo "5"
cmake ..
make
sudo make install
 . /usr/share/gazebo/setup.sh
 . /usr/share/mavlink_sitl_gazebo/setup.sh
cd ~/src/Firmware
