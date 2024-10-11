#!/bin/bash

if [[ -z "${DOCKER_TAG}" ]]; then
  # The default tag name should be hardcoded
  # to whatever we are currently using in CI on this branch
  TAG_NAME="2024-10-11"
else
  TAG_NAME="${DOCKER_TAG}"
fi

# set the image name and tag
PX4_DOCKER_REPO="ghcr.io/px4/px4-dev:$TAG_NAME"
# get the project root path
SRC_DIR=$(dirname $(realpath $0))
SRC_DIR=${SRC_DIR%/*/*}

echo "[docker_run.sh] Project Root: $SRC_DIR"
echo "[docker_run.sh] ⭐️ Running: '$PX4_DOCKER_REPO'"

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
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --volume=/tmp:/tmp:rw \
  --volume=${SRC_DIR}:${SRC_DIR}:rw \
  ${PX4_DOCKER_REPO} /bin/bash -c "$1 $2 $3"
