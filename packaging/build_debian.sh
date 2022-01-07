#!/bin/bash

# usage: ./build_debian.sh dest_dir

set -e

if [ "$1" = "" ]; then
	echo "ERROR: Package output directory not given"
	echo " usage: $0 <output-dir> [no-rebuild]"
	echo "   output-dir : directory for output artifacts (deb and elf files)"
	echo "   no-rebuild : flag to disable px4-firmware re-building, but use existing docker artifact images instead"
	exit 1
fi

script_dir=$(dirname $(realpath $0))
dest_dir=$(realpath $1)

no_rebuild=$2

mkdir -p ${dest_dir}

# Generate debian package
pushd ${script_dir}

iname_p=tii-px4-pixhawk-artifacts
iname_s=tii-px4-saluki-artifacts
if [ "${no_rebuild}" != "" ]; then
	[ "$(docker images | grep tii-px4-pixhawk-artifacts)" == "" ] && docker build -t ${iname_p} -f Dockerfile.pixhawk ..
	[ "$(docker images | grep tii-px4-saluki-artifacts)" == "" ] && docker build -t ${iname_s} -f Dockerfile.saluki ..
else
	docker build -t ${iname_p} -f Dockerfile.pixhawk ..
	docker build -t ${iname_s} -f Dockerfile.saluki ..
fi

version=$(git describe --always --tags --dirty | sed 's/^v//')
iname=tii-px4-debian-artifacts
docker build -t ${iname} -f Dockerfile.debian \
	--build-arg VERSION=${version} \
	--build-arg PACKAGING_PATH=packaging \
	..

container_id=$(docker create ${iname} "")
mkdir -p tmp_
pushd tmp_
docker cp ${container_id}:/artifacts .
docker rm ${container_id}
cp artifacts/* ${dest_dir}
popd
rm -Rf tmp_

popd


echo "Done"
