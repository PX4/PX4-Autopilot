#! /bin/bash

if [ -z "$PX4_DOCKER_REPO" ]; then
	PX4_DOCKER_REPO=px4io/px4-dev-nuttx:2017-01-28
fi

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

CCACHE_DIR=${HOME}/.ccache
mkdir -p ${CCACHE_DIR}

X11_TMP=/tmp/.X11-unix

docker run -it --rm -w ${SRC_DIR} \
	-e AWS_ACCESS_KEY_ID=${AWS_ACCESS_KEY_ID} \
	-e AWS_SECRET_ACCESS_KEY=${AWS_SECRET_ACCESS_KEY} \
	-e BRANCH_NAME=${BRANCH_NAME} \
	-e CCACHE_DIR=${CCACHE_DIR} \
	-e CI=${CI} \
	-e DISPLAY=$DISPLAY \
	-e GIT_SUBMODULES_ARE_EVIL=1 \
	-e LOCAL_USER_ID=`id -u` \
	-e TRAVIS_BRANCH=${TRAVIS_BRANCH} \
	-e TRAVIS_BUILD_ID=${TRAVIS_BUILD_ID} \
	-v ${CCACHE_DIR}:${CCACHE_DIR}:rw \
	-v ${SRC_DIR}:${SRC_DIR}:rw \
	-v ${X11_TMP}:${X11_TMP}:ro \
	${PX4_DOCKER_REPO} /bin/bash -c "$@"
