#!/bin/bash

source "/home/dennis/src/PX4/Firmware/Tools/setup_gazebo.bash" "/home/dennis/src/PX4/Firmware/" "/home/dennis/src/PX4/Firmware/build/px4_sitl_default"
./run_tests.py
