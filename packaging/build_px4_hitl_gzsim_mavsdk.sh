#!/bin/bash

source /opt/ros/humble/setup.sh

pushd px4-firmware
	rm -rf build
	make clean
	make px4_sitl_default
	make px4_sitl mavsdk_tests
	make px4_sitl gzsim-plugins
popd
