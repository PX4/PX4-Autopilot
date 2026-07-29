#!/bin/bash

set -e

export PX4_HOME_LAT=49.796766
export PX4_HOME_LON=24.347826
export PX4_HOME_ALT=270
export PX4_CMAKE_BUILD_TYPE=Debug

cd ..

rm -f build/px4_sitl_sih/rootfs/parameters.bson build/px4_sitl_sih/rootfs/parameters_backup.bson

make px4_sitl_sih sihsim_quadx
