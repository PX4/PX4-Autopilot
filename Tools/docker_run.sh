#! /bin/bash

if [ -z ${PX4_DOCKER_REPO+x} ]; then
	PX4_DOCKER_REPO="px4io/px4-dev:v1.17.0-beta1"
else
	echo "PX4_DOCKER_REPO is set to '$PX4_DOCKER_REPO'";
fi

echo "PX4_DOCKER_REPO: $PX4_DOCKER_REPO";

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=${SCRIPT_DIR}/../

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

docker run -it --rm -w "${SRC_DIR}" \
	--user="$(id -u):$(id -g)" \
	--env=CCACHE_DIR="${CCACHE_DIR}" \
	--env=PX4_ASAN \
	--env=PX4_MSAN \
	--env=PX4_TSAN \
	--env=PX4_UBSAN \
	--publish 14556:14556/udp \
	--volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
	--volume=${SRC_DIR}:${SRC_DIR}:rw \
	${PX4_DOCKER_REPO} /bin/bash -c "$@"
