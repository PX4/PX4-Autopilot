#!/bin/bash

# Usage: ./build_sitl_container.sh <image>

set -e

script_dir=$(dirname $(realpath $0))
iname=${1:-ghcr.io/tiiuae/tii-px4-sitl}

# Generate binary
pushd ${script_dir}
version=$(git describe --always --tags --dirty | sed 's/^v//')
docker build -t ${iname} -f Dockerfile.docker \
	--build-arg FROM_ARTIFACTS=tii-px4-sitl-artifacts \
	..
popd
