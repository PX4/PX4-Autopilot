#!/bin/bash

if [[ -z "${DOCKER_TAG}" ]]; then
  # the default tag for docker images
  # follows the pattern below, and we recommend
  # that any images pushed to the registry continue
  # to use the pattern for consistency
  TAG_NAME="`date +"%Y-%m-%d"`"
else
  TAG_NAME="${DOCKER_TAG}"
fi

# container name and tag
PX4_DOCKER_REPO="ghcr.io/px4/px4-dev:$TAG_NAME"
# get project root directory
SRC_DIR=$(dirname $(realpath $0))
SRC_DIR=${SRC_DIR%/*/*}

echo "[docker_build.sh] Source: [$SRC_DIR]"
echo "[docker_build.sh] ⭐️ Building: [$PX4_DOCKER_REPO]"

docker build \
  -t ${PX4_DOCKER_REPO} \
  -f Tools/container/Dockerfile "${SRC_DIR}"

# push to the container registry
if [[ "${PUSH_TO_REGISTRY}" ]]; then
  docker push ${PX4_DOCKER_REPO}
fi
