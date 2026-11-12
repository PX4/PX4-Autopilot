#!/usr/bin/env bash
# Copy msgs and the message translation node into a ROS workspace directory

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

PX4_SRC_DIR="$DIR/.."

WS_DIR="$1"
EXTERNAL_MODULES_LOCATION="$2"
if [ ! -e "${WS_DIR}" ]
then
	echo "Usage: $0 <ros_ws_dir> [<external_modules_location>]"
	exit 1
fi
WS_DIR="$WS_DIR"/src
if [ ! -e "${WS_DIR}" ]
then
	echo "'src' directory not found inside ROS workspace (${WS_DIR})"
	exit 1
fi

# Out-of-tree messages join px4_msgs too, so none may have the name of a PX4 message
if [ -n "${EXTERNAL_MODULES_LOCATION}" ]
then
	EXTERNAL_MSG_DIR="${EXTERNAL_MODULES_LOCATION}"/msg
	if [ ! -d "${EXTERNAL_MSG_DIR}" ]
	then
		echo "'msg' directory not found inside the external modules location (${EXTERNAL_MSG_DIR})"
		exit 1
	fi
	for msg in "${EXTERNAL_MSG_DIR}"/*.msg
	do
		name=$(basename "${msg}")
		if [ -e "${PX4_SRC_DIR}/msg/${name}" ] || [ -e "${PX4_SRC_DIR}/msg/versioned/${name}" ]
		then
			echo "${EXTERNAL_MSG_DIR}/${name} has the name of a PX4 message"
			exit 1
		fi
	done
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
if [ -n "${EXTERNAL_MODULES_LOCATION}" ]
then
	cp -a "${EXTERNAL_MSG_DIR}"/*.msg "${PX4_MSGS_DIR}"/msg
fi
