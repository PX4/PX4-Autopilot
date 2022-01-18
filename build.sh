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


version=$(git describe --always --tags --dirty | sed 's/^v//')

# Build Pixhawk4 image
docker run \
  --rm \
  -v ${script_dir}:/px4-firmware/sources \
  ${iname_env} \
  ./packaging/build_pixhawk4.sh

# Build Saluki image
docker run \
  --rm \
  -v ${script_dir}:/px4-firmware/sources \
  ${iname_env} \
  ./packaging/build_saluki.sh

# Generate debian package
docker run \
  --rm \
  -v ${script_dir}:/px4-firmware/sources \
  ${iname_env} \
  ./packaging/build_px4fwupdater.sh \
  -v ${version}

# Copy artifacts to destination directory
cp ${script_dir}/build/px4_fmu-v5_ssrc/px4_fmu-v5_ssrc.px4                      ${dest_dir}/px4_fmu-v5_ssrc-${version}.px4
cp ${script_dir}/build/px4_fmu-v5x_ssrc/px4_fmu-v5x_ssrc.px4                    ${dest_dir}/px4_fmu-v5x_ssrc-${version}.px4
cp ${script_dir}/build/ssrc_saluki-v1_default/ssrc_saluki-v1_default.px4        ${dest_dir}/ssrc_saluki-v1_default-${version}.px4
cp ${script_dir}/build/ssrc_saluki-v1_bootloader/ssrc_saluki-v1_bootloader.elf  ${dest_dir}/ssrc_saluki-v1_bootloader-${version}.elf
mv ${script_dir}/px4fwupdater*.deb                                              ${dest_dir}/

echo "Done"
exit 0
