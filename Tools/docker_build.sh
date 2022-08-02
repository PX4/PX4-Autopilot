#!/bin/bash

if [[ -z "${DOCKER_TAG}" ]]; then
  TAG_NAME="latest"
else
  TAG_NAME="${DOCKER_TAG}"
fi

PX4_DOCKER_REPO="px4io/px4-dev:$TAG_NAME"
BUILD_ARGS="${@}"
PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

echo "[docker_build.sh]: Building [$PX4_DOCKER_REPO]"
echo "[docker_build.sh]:  - with args: [$BUILD_ARGS]"

docker build \
  -t ${PX4_DOCKER_REPO} \
  -f Tools/setup/Dockerfile "${SRC_DIR}" \
  --build-arg INSTALL_ARGS="${BUILD_ARGS}"
