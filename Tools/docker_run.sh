#! /bin/bash

if [[ $@ =~ .*px4fmu.* ]]; then
	# nuttx-px4fmu-v{1,2,3,4,5}
	PX4_DOCKER_REPO="px4io/px4-dev-nuttx:2017-06-01"
elif [[ $@ =~ .*rpi.* ]] || [[ $@ =~ .*bebop.* ]]; then
	# posix_rpi_cross, posix_bebop_default
	PX4_DOCKER_REPO="px4io/px4-dev-raspi:2017-04-22"
elif [[ $@ =~ .*eagle.* ]] || [[ $@ =~ .*excelsior.* ]]; then
	# eagle, excelsior
	PX4_DOCKER_REPO="lorenzmeier/px4-dev-snapdragon:2017-01-14"
elif [[ $@ =~ .*clang.* ]] || [[ $@ =~ .*scan-build.* ]]; then
	# clang tools
	PX4_DOCKER_REPO="px4io/px4-dev-clang:2017-04-22"
elif [[ $@ =~ .*cppcheck.* ]]; then
	# need Ubuntu 17.04 for cppcheck cmake support
	# TODO: remove this once px4io/px4-dev-base updates
	PX4_DOCKER_REPO=px4io/px4-dev-base:ubuntu17.04
elif [[ $@ =~ .*tests* ]]; then
	# run all tests with simulation
	PX4_DOCKER_REPO="px4io/px4-dev-simulation:2017-06-01"
fi

# otherwise default to nuttx
if [ -z "$PX4_DOCKER_REPO" ]; then
	PX4_DOCKER_REPO="px4io/px4-dev-nuttx:2017-06-01"
fi

# docker hygiene

#Delete all stopped containers (including data-only containers)
#docker rm $(docker ps -a -q)

#Delete all 'untagged/dangling' (<none>) images
#docker rmi $(docker images -q -f dangling=true)

echo "PX4_DOCKER_REPO: $PX4_DOCKER_REPO";

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

X11_TMP=/tmp/.X11-unix

docker run -it --rm -w "${SRC_DIR}" \
	-e AWS_ACCESS_KEY_ID="${AWS_ACCESS_KEY_ID}" \
	-e AWS_SECRET_ACCESS_KEY="${AWS_SECRET_ACCESS_KEY}" \
	-e BRANCH_NAME="${BRANCH_NAME}" \
	-e CCACHE_DIR="${CCACHE_DIR}" \
	-e CI="${CI}" \
	-e CODECOV_TOKEN="${CODECOV_TOKEN}" \
	-e COVERALLS_REPO_TOKEN="${COVERALLS_REPO_TOKEN}" \
	-e DISPLAY="${DISPLAY}" \
	-e LOCAL_USER_ID="$(id -u)" \
	-e TRAVIS_BRANCH="${TRAVIS_BRANCH}" \
	-e TRAVIS_BUILD_ID="${TRAVIS_BUILD_ID}" \
	-v ${CCACHE_DIR}:${CCACHE_DIR}:rw \
	-v ${SRC_DIR}:${SRC_DIR}:rw \
	-v ${X11_TMP}:${X11_TMP}:ro \
	${PX4_DOCKER_REPO} /bin/bash -c "$@"
