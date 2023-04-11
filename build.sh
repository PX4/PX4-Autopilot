#!/bin/bash -eux

usage() {
  set +x
  echo ""
	echo " usage: $0 <output-dir> <build-target>"
	echo "   output-dir : directory for output artifacts"
	echo "   build-target : supported build targets:"
  echo "     px4fwupdater"
  echo "     pixhawk"
  echo "     saluki-v1_default"
  echo "     saluki-v1_protected"
  echo "     saluki-v1_amp"
  echo "     saluki-v1_bootloader"
  echo "     saluki-v2_default"
  echo "     saluki-v2_amp"
  echo "     saluki-v2_bootloader"
  echo "     saluki-v2_protected"
  echo "     saluki-pi_default"
  echo "     saluki-pi_amp"
  echo "     saluki-pi_bootloader"
  echo "     saluki-pi_protected"
  echo
  exit 1
}

dest_dir="${1:-}"
target="${2:-}"

if [ -z "$dest_dir" ]; then
  usage
fi

version=$(git describe --always --tags --dirty | sed 's/^v//')
script_dir=$(dirname $(realpath $0))
dest_dir=$(realpath $1)
iname_env=tii_px4_build

mkdir -p ${dest_dir}
pushd ${script_dir}

build_env="docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) --pull -f ./packaging/Dockerfile.build_env -t ${iname_env} ."
build_cmd_fw="docker run --rm -v ${script_dir}:/px4-firmware/sources ${iname_env} ./packaging/build_px4fw.sh"
build_cmd_px4fwupdater="${script_dir}/packaging/build_px4fwupdater.sh -v ${version} -i ${dest_dir}"

# Generate build_env
if [ "${target}" != px4fwupdater ]; then
  $build_env
fi

case $target in
  "px4fwupdater")
    $build_cmd_px4fwupdater
    ;;
  "pixhawk")
    $build_cmd_fw px4_fmu-v5x_ssrc
    cp ${script_dir}/build/px4_fmu-v5x_ssrc/px4_fmu-v5x_ssrc.px4 ${dest_dir}/px4_fmu-v5x_ssrc-${version}.px4
    ;;
  "saluki-v1_default")
    $build_cmd_fw ssrc_saluki-v1_default
    cp ${script_dir}/build/ssrc_saluki-v1_default/ssrc_saluki-v1_default.px4 ${dest_dir}/ssrc_saluki-v1_default-${version}.px4
    ;;
  "saluki-v1_protected")
    $build_cmd_fw ssrc_saluki-v1_protected
    cp ${script_dir}/build/ssrc_saluki-v1_protected/ssrc_saluki-v1_protected.px4 ${dest_dir}/ssrc_saluki-v1_protected-${version}.px4
    ;;
  "saluki-v1_amp")
    $build_cmd_fw ssrc_saluki-v1_amp
    cp ${script_dir}/build/ssrc_saluki-v1_amp/ssrc_saluki-v1_amp.bin ${dest_dir}/ssrc_saluki-v1_amp-${version}.bin
    ;;
  "saluki-v1_bootloader")
    $build_cmd_fw ssrc_saluki-v1_bootloader
    cp ${script_dir}/build/ssrc_saluki-v1_bootloader/ssrc_saluki-v1_bootloader.elf ${dest_dir}/ssrc_saluki-v1_bootloader-${version}.elf
    ;;
  "saluki-v2_default")
    $build_cmd_fw ssrc_saluki-v2_default
    cp ${script_dir}/build/ssrc_saluki-v2_default/ssrc_saluki-v2_default.px4 ${dest_dir}/ssrc_saluki-v2_default-${version}.px4
    ;;
  "saluki-v2_protected")
    $build_cmd_fw ssrc_saluki-v2_protected
    cp ${script_dir}/build/ssrc_saluki-v2_protected/ssrc_saluki-v2_protected.px4 ${dest_dir}/ssrc_saluki-v2_protected-${version}.px4
    ;;
  "saluki-v2_amp")
    $build_cmd_fw ssrc_saluki-v2_amp
    cp ${script_dir}/build/ssrc_saluki-v2_amp/ssrc_saluki-v2_amp.bin ${dest_dir}/ssrc_saluki-v2_amp-${version}.bin
    ;;
  "saluki-v2_bootloader")
    $build_cmd_fw ssrc_saluki-v2_bootloader
    cp ${script_dir}/build/ssrc_saluki-v2_bootloader/ssrc_saluki-v2_bootloader.elf ${dest_dir}/ssrc_saluki-v2_bootloader-${version}.elf
    ;;
  "saluki-pi_default")
    $build_cmd_fw ssrc_saluki-pi_default
    cp ${script_dir}/build/ssrc_saluki-pi_default/ssrc_saluki-pi_default.px4 ${dest_dir}/ssrc_saluki-pi_default-${version}.px4
    ;;
  "saluki-pi_protected")
    $build_cmd_fw ssrc_saluki-pi_protected
    cp ${script_dir}/build/ssrc_saluki-pi_protected/ssrc_saluki-pi_protected.px4 ${dest_dir}/ssrc_saluki-pi_protected-${version}.px4
    ;;
  "saluki-pi_amp")
    $build_cmd_fw ssrc_saluki-pi_amp
    cp ${script_dir}/build/ssrc_saluki-pi_amp/ssrc_saluki-pi_amp.bin ${dest_dir}/ssrc_saluki-pi_amp-${version}.bin
    ;;
  "saluki-pi_bootloader")
    $build_cmd_fw ssrc_saluki-pi_bootloader
    cp ${script_dir}/build/ssrc_saluki-pi_bootloader/ssrc_saluki-pi_bootloader.elf ${dest_dir}/ssrc_saluki-pi_bootloader-${version}.elf
    ;;
   *)
    usage
    ;;
esac

echo "Done"

