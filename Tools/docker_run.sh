#! /bin/bash

# docker hygiene

#Delete all stopped containers (including data-only containers)
#docker rm $(docker ps -a -q)

#Delete all 'untagged/dangling' (<none>) images
#docker rmi $(docker images -q -f dangling=true)

PX4_DOCKER_REPO="px4io/px4-dev"

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

docker run -it --rm -w "${SRC_DIR}" \
  --env=LOCAL_USER_ID="$(id -u)" \
  --publish 14556:14556/udp \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --volume=/tmp:/tmp:rw \
  --volume=${SRC_DIR}:${SRC_DIR}:rw \
  ${PX4_DOCKER_REPO} /bin/bash -c "$1 $2 $3"
