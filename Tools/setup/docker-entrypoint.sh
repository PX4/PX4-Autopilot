#!/bin/bash

GREEN='\033[0;32m'
NO_COLOR='\033[0m' # No Color
SCRIPTID="${GREEN}[docker-entrypoint.sh]${NO_COLOR}"

echo -e "$SCRIPTID Starting"

# Start virtual X server in the background
# - DISPLAY default is :99, set in dockerfile
# - Users can override with `-e DISPLAY=` in `docker run` command to avoid
#   running Xvfb and attach their screen
if [[ -x "$(command -v Xvfb)" && "$DISPLAY" == ":99" ]]; then
	echo -e "$SCRIPTID Starting Xvfb"
	Xvfb :99 -screen 0 1600x1200x24+32 &
fi

# Check if the ROS_DISTRO is passed and use it
# to source the ROS environment
if [ -n "${ROS_DISTRO}" ]; then
	echo -e "$SCRIPTID ROS: ${ROS_DISTRO}"
	source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

echo -e "$SCRIPTID ($( uname -m ))"

exec "$@"
