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

# Pulling latest image
# TODO: remove old one to save space, until then, update manually
#echo "===> pull latest Docker image"
#docker pull px4io/sitl-testing
#echo "<==="

#
# Running SITL testing container
# Assuming that necessary source projects, including this one, are cloned in the build server workspace of this job.
#
echo "===> run container"
docker run --rm -v "$WORKSPACE:/job:rw" px4io/px4-dev-ros bash "/job/Firmware/integrationtests/run-tests.bash"
echo "<==="
