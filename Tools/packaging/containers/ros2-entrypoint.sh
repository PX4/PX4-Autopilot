#!/usr/bin/env bash
set -e

source /usr/local/lib/px4/network.sh
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/local_setup.bash

exec "$@"
