#! /bin/bash

if [ -z ${PX4_DOCKER_REPO+x} ]; then
	echo "guessing PX4_DOCKER_REPO based on input";
	if [[ $@ =~ .*px4_fmu.* ]]; then
		# nuttx-px4fmu-v{1,2,3,4,5}
		PX4_DOCKER_REPO="px4io/px4-dev-nuttx:2018-11-22"
	elif [[ $@ =~ .*navio2.* ]] || [[ $@ =~ .*raspberry.* ]] || [[ $@ =~ .*bebop.* ]]; then
		# posix_rpi_cross, posix_bebop_default
		PX4_DOCKER_REPO="px4io/px4-dev-raspi:2018-11-22"
	elif [[ $@ =~ .*eagle.* ]] || [[ $@ =~ .*excelsior.* ]]; then
		# eagle, excelsior
		PX4_DOCKER_REPO="lorenzmeier/px4-dev-snapdragon:2018-09-12"
	elif [[ $@ =~ .*ocpoc.* ]]; then
		# aerotennaocpoc_ubuntu
		PX4_DOCKER_REPO="px4io/px4-dev-armhf:2018-11-22"
	elif [[ $@ =~ .*clang.* ]] || [[ $@ =~ .*scan-build.* ]]; then	
		# clang tools
		PX4_DOCKER_REPO="px4io/px4-dev-clang:2018-11-22"
	elif [[ $@ =~ .*cppcheck.* ]]; then
		# TODO: remove this once px4io/px4-dev-base updates
		PX4_DOCKER_REPO="px4io/px4-dev-base:2018-11-22"
	elif [[ $@ =~ .*tests* ]]; then
		# run all tests with simulation
		PX4_DOCKER_REPO="px4io/px4-dev-simulation:2018-11-22"
	fi
else
	echo "PX4_DOCKER_REPO is set to '$PX4_DOCKER_REPO'";
fi

# otherwise default to nuttx
if [ -z ${PX4_DOCKER_REPO+x} ]; then
	PX4_DOCKER_REPO="px4io/px4-dev-nuttx:2018-11-22"
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

docker run -it --rm -w "${SRC_DIR}" \
	--env=AWS_ACCESS_KEY_ID \
	--env=AWS_SECRET_ACCESS_KEY \
	--env=BRANCH_NAME \
	--env=CCACHE_DIR="${CCACHE_DIR}" \
	--env=CI \
	--env=CODECOV_TOKEN \
	--env=COVERALLS_REPO_TOKEN \
	--env=LOCAL_USER_ID="$(id -u)" \
	--env=PX4_ASAN \
	--env=PX4_MSAN \
	--env=PX4_TSAN \
	--env=PX4_UBSAN \
	--env=TRAVIS_BRANCH \
	--env=TRAVIS_BUILD_ID \
	--publish 14556:14556/udp \
	--volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
	--volume=${SRC_DIR}:${SRC_DIR}:rw \
	${PX4_DOCKER_REPO} /bin/bash -c "$1 $2 $3"
