#!/bin/bash

source /opt/ros/galactic/setup.bash

px4 -d "/px4_sitl_etc" -w sitl_${PX4_SIM_MODEL} -s /px4_sitl_etc/init.d-posix/rcS
