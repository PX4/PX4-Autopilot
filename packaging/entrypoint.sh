#!/bin/bash

source /opt/ros/humble/setup.bash

if [ "$1" == "sim_tcp_server" ] || [ "$PX4_SIM_USE_TCP_SERVER" != "" ]; then
    px4 -d "/px4_sitl_etc" -w sitl_${PX4_SIM_MODEL} -s /px4_sitl_etc/init.d-posix/rcS.sim_tcp_server
elif [ "$1" == "sim_udp" ] || [ "$PX4_SIM_USE_UDP" != "" ]; then
    px4 -d "/px4_sitl_etc" -w sitl_${PX4_SIM_MODEL} -s /px4_sitl_etc/init.d-posix/rcS.sim_udp
else
    px4 -d "/px4_sitl_etc" -w sitl_${PX4_SIM_MODEL} -s /px4_sitl_etc/init.d-posix/rcS
fi
