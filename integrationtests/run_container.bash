#!/bin/bash
#
# Run container and start test execution
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository
set -e

if [ -z "$WORKSPACE" ]; then
    echo "\$WORKSPACE not set"
    exit 1
fi

IMAGE=px4io/px4-dev-ros:v1.0

# Pulling latest image
echo "===> pull latest Docker image"
docker pull $IMAGE

# removing some images might fail
set +e
docker rmi $(docker images --filter "dangling=true" -q --no-trunc)
set -e
echo "<==="

#
# Running SITL testing container
# Assuming that necessary source projects, including this one, are cloned in the build server workspace of this job.
#
echo "===> run container"
docker run --rm -v "$WORKSPACE:/job:rw" $IMAGE bash "/job/Firmware/integrationtests/run_tests.bash" /job/Firmware
echo "<==="
