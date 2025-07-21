#! /bin/bash
# Copy msgs and the message translation node into a ROS workspace directory

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

PX4_SRC_DIR="$DIR/.."

WS_DIR="$1"
if [ ! -e "${WS_DIR}" ]
then
	echo "Usage: $0 <ros_ws_dir>"
	exit 1
fi
WS_DIR="$WS_DIR"/src
if [ ! -e "${WS_DIR}" ]
then
	echo "'src' directory not found inside ROS workspace (${WS_DIR})"
	exit 1
fi

cp -ar "${PX4_SRC_DIR}"/msg/translation_node "${WS_DIR}"
cp -ar "${PX4_SRC_DIR}"/msg/px4_msgs_old "${WS_DIR}"
PX4_MSGS_DIR="${WS_DIR}"/px4_msgs
if [ ! -e "${PX4_MSGS_DIR}" ]
then
  git clone https://github.com/PX4/px4_msgs.git "${PX4_MSGS_DIR}"
  rm -rf "${PX4_MSGS_DIR}"/msg/*.msg
  rm -rf "${PX4_MSGS_DIR}"/msg/versioned/*.msg
  rm -rf "${PX4_MSGS_DIR}"/srv/*.srv
fi
cp -ar "${PX4_SRC_DIR}"/msg/*.msg "${PX4_MSGS_DIR}"/msg
cp -ar "${PX4_SRC_DIR}"/msg/versioned/*.msg "${PX4_MSGS_DIR}"/msg
cp -ar "${PX4_SRC_DIR}"/srv/*.srv "${PX4_MSGS_DIR}"/srv
