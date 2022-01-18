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

pushd ${script_dir}

# Generate build_env
iname_env=tii_px4_build
docker build \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  --pull \
  -f ./packaging/Dockerfile.build_env -t ${iname_env} .

# Build Saluki image
version=$(git describe --always --tags --dirty | sed 's/^v//')

docker run \
  --rm \
  -v ${script_dir}:/px4-firmware/sources \
  ${iname_env} \
  ./packaging/build_px4_sitl.sh \
    -v ${version} \

mv px4_sitl_build-*.tar.gz   ${dest_dir}/
mv px4_gazebo_data-*.tar.gz  ${dest_dir}/

echo "Done"
exit 0
