#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m' # No Color
FILE_DESCRIPTOR="${GREEN}[docker-entrypoint.sh]${NC}"

echo -e "$FILE_DESCRIPTOR Starting"

# Start virtual X server in the background
# - DISPLAY default is :99, set in dockerfile
# - Users can override with `-e DISPLAY=` in `docker run` command to avoid
#   running Xvfb and attach their screen
if [[ -x "$(command -v Xvfb)" && "$DISPLAY" == ":99" ]]; then
	echo -e "$FILE_DESCRIPTOR Starting Xvfb"
	Xvfb :99 -screen 0 1600x1200x24+32 &
fi

# Check if the ROS_DISTRO is passed and use it
# to source the ROS environment
if [ -n "${ROS_DISTRO}" ]; then
	echo -e "$FILE_DESCRIPTOR ROS: ${ROS_DISTRO}"
	source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

echo -e "$FILE_DESCRIPTOR ($( uname -m ))"

exec "$@"
