#! /bin/bash

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

CCACHE_DIR=${HOME}/.ccache
mkdir -p ${CCACHE_DIR}

docker run -it --rm -w ${SRC_DIR} \
	-v ${SRC_DIR}:${SRC_DIR}:rw \
	-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	-v ${CCACHE_DIR}:${CCACHE_DIR}:rw \
	-e CCACHE_DIR=${CCACHE_DIR} \
	-e LOCAL_USER_ID=`id -u` \
	px4io/px4-dev-nuttx:2017-01-08 /bin/bash -c "$@"
