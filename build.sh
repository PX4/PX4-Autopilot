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

# Generate debian package
pushd ${script_dir}/packaging

version=$(git describe --always --tags --dirty | sed 's/^v//')

iname_p=tii-px4-pixhawk-artifacts
docker build -t ${iname_p} -f Dockerfile.build_pixhawk \
	--build-arg VERSION=${version} \
	..

iname_s=tii-px4-saluki-artifacts
docker build -t ${iname_s} -f Dockerfile.build_saluki \
	--build-arg VERSION=${version} \
	..

iname=tii-px4-debian-artifacts
docker build -t ${iname} -f Dockerfile.build_px4fwupdater \
	--build-arg VERSION=${version} \
	..

container_deb=$(docker create ${iname} "")
container_p=$(docker create ${iname_p} "")
container_s=$(docker create ${iname_s} "")
mkdir -p tmp_
pushd tmp_
docker cp ${container_deb}:/artifacts .
docker rm ${container_deb}
docker cp ${container_p}:/artifacts .
docker rm ${container_p}
docker cp ${container_s}:/artifacts .
docker rm ${container_s}
cp artifacts/* ${dest_dir}
popd
rm -Rf tmp_

popd


echo "Done"
exit 0
