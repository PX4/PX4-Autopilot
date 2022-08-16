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

PX4_DOCKER_REPO="ghcr.io/px4/px4-dev:$TAG_NAME"
PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

echo "[docker_build.sh]: Building [$PX4_DOCKER_REPO]"

docker build \
  -t ${PX4_DOCKER_REPO} \
  -f Tools/setup/Dockerfile "${SRC_DIR}"
