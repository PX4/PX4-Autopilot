#!/bin/sh
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

# main ROS Setup
# following http://wiki.ros.org/indigo/Installation/Ubuntu
# also adding drcsim http://gazebosim.org/tutorials?tut=drcsim_install
# run this file with . ./px4_ros_setup_ubuntu.sh

## add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

## add key
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | \
    sudo apt-key add -

## Install main ROS pacakges
sudo apt-get update
sudo apt-get -y install ros-indigo-desktop-full
sudo rosdep init
rosdep update

## Setup environment variables
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
. ~/.bashrc

# get rosinstall
sudo apt-get -y install python-rosinstall

# additional dependencies
sudo apt-get -y install ros-indigo-octomap-msgs ros-indigo-joy

## drcsim setup (for models)
### add osrf repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu trusty main" > /etc/apt/sources.list.d/drc-latest.list'

### add key
wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -

### install drcsim
sudo apt-get update
sudo apt-get -y install drcsim
