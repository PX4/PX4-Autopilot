#!/bin/bash

set -euxo pipefail

if [ "$1" = "" ]; then
	echo "ERROR: Package output directory not given"
	echo " usage: $0 <output-dir>"
	echo "   output-dir : directory for output artifacts"
	exit 1
fi

script_dir=$(dirname $(realpath $0))
dest_dir=$(realpath $1)

mkdir -p ${dest_dir}

# Generate binary
pushd ${script_dir}/packaging

version=$(git describe --always --tags --dirty | sed 's/^v//')
iname=${IMAGE_NAME:-tii-px4-sitl-artifacts}
docker build -t ${iname} -f Dockerfile.sitl \
	--build-arg VERSION=${version} \
	..

container_id=$(docker create ${iname} "")
mkdir -p tmp_
pushd tmp_
docker cp ${container_id}:/artifacts .
docker rm ${container_id}
mkdir -p ${dest_dir}
cp artifacts/* ${dest_dir}
popd
rm -Rf tmp_
