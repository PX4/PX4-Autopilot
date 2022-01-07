#!/bin/bash

# usage: ./build_pixhawk.sh dest_dir

set -e

if [ "$1" = "" ]; then
	echo "ERROR: Package output directory not given"
	echo " usage: $0 <output-dir>"
	exit 1
fi

script_dir=$(dirname $(realpath $0))
dest_dir=$(realpath $1)

mkdir -p ${dest_dir}

# Generate binary
pushd ${script_dir}
iname=tii-px4-pixhawk-artifacts
docker build -t ${iname} -f Dockerfile.pixhawk ..
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
