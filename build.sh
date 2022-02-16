#!/bin/bash

usage() {
	echo " usage: $0 <output-dir> <build-target>"
	echo "   output-dir : directory for output artifacts"
	echo "   build-target : supported build targets pixhawk4, saluki-v1, px4fwupdater"
}

dest_dir="${1:-}"
target="${2:-all}"

if [ -z "$dest_dir" ]; then
	echo "ERROR: Package output directory not given"
  usage
	exit 1
fi

if [[ ! "${target}" =~ all|pixhawk4|saluki-v1|px4fwupdater ]]
then
	echo "ERROR: build target ''${target}'' not supported!"
  usage
  exit 1
fi

set -euxo pipefail

script_dir=$(dirname $(realpath $0))
dest_dir=$(realpath $1)

mkdir -p ${dest_dir}

pushd ${script_dir}

# Generate build_env
if [ "${target}" != px4fwupdater ]
then
  iname_env=tii_px4_build
  docker build \
    --build-arg UID=$(id -u) \
    --build-arg GID=$(id -g) \
    --pull \
    -f ./packaging/Dockerfile.build_env -t ${iname_env} .
fi

version=$(git describe --always --tags --dirty | sed 's/^v//')

# Build Pixhawk4 image
if [ "${target}" == all ] || [ "${target}" == pixhawk4 ]
then
  docker run \
    --rm \
    -v ${script_dir}:/px4-firmware/sources \
    ${iname_env} \
    ./packaging/build_pixhawk4.sh
  cp ${script_dir}/build/px4_fmu-v5_ssrc/px4_fmu-v5_ssrc.px4                      ${dest_dir}/px4_fmu-v5_ssrc-${version}.px4
  cp ${script_dir}/build/px4_fmu-v5x_ssrc/px4_fmu-v5x_ssrc.px4                    ${dest_dir}/px4_fmu-v5x_ssrc-${version}.px4
fi

# Build Saluki image
if [ "${target}" == all ] || [ "${target}" == saluki-v1 ]
then
  docker run \
    --rm \
    -v ${script_dir}:/px4-firmware/sources \
    ${iname_env} \
    ./packaging/build_saluki.sh
  cp ${script_dir}/build/ssrc_saluki-v1_default/ssrc_saluki-v1_default.px4        ${dest_dir}/ssrc_saluki-v1_default-${version}.px4
  cp ${script_dir}/build/ssrc_saluki-v1_bootloader/ssrc_saluki-v1_bootloader.elf  ${dest_dir}/ssrc_saluki-v1_bootloader-${version}.elf
fi

# Generate debian package
if [ "${target}" == all ] || [ "${target}" == px4fwupdater ]
then
  #docker run \
  #  --rm \
  #  -v ${script_dir}:/px4-firmware/sources \
  #  ${iname_env} \
  #  ./packaging/build_px4fwupdater.sh \
  #  -v ${version} \
  #  -i ${dest_dir}
  "${script_dir}"/packaging/build_px4fwupdater.sh -v "${version}" -i "${dest_dir}"
fi

echo "Done"
