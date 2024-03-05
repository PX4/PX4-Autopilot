#!/bin/bash

cd Tools/simulation/gazebo-classic/sitl_gazebo-classic/src
patch < ../../../../../boards/modalai/voxl2/gazebo-docker/patch/mavlink_interface.patch
cd -

cd Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl
patch < ../../../../../../boards/modalai/voxl2/gazebo-docker/patch/iris_hitl.patch
cd -

cd Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_vision
patch < ../../../../../../boards/modalai/voxl2/gazebo-docker/patch/iris_vision.patch
cd -