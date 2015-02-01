#!/bin/sh
# this script creates a catkin_ws in the current folder
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
