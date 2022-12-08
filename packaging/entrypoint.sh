#!/bin/bash

source /opt/ros/galactic/setup.bash

if [ "$1" == "sim_tcp_server" ]; then
    px4 -d "/px4_sitl_etc" -w sitl_${PX4_SIM_MODEL} -s /px4_sitl_etc/init.d-posix/rcS.sim_tcp_server
else
    px4 -d "/px4_sitl_etc" -w sitl_${PX4_SIM_MODEL} -s /px4_sitl_etc/init.d-posix/rcS
fi
