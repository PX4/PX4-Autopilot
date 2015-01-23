#!/bin/sh
# this script creates a catkin_ws in the current folder

mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
