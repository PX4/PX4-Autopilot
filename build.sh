#!/bin/bash -eux

usage() {
  set +x
  echo ""
  echo " usage: $0 <output-dir> <build-target>"
  echo "   output-dir : directory for output artifacts"
  echo "   build-target : supported build targets:"
  echo "     px4fwupdater"
  echo "     pixhawk"
  echo "     fmu-v6xrt"
  echo "     saluki-v1_default"
  echo "     saluki-v2_default"
  echo "     saluki-v2_amp"
  echo "     saluki-v2_flat"
  echo "     saluki-pi_default"
  echo "     saluki-pi_amp"
  echo "     saluki-pi_flat"
  echo "     saluki-v3_default"
  echo "     saluki-v3_amp"
  echo "     saluki-v3_flat"
  echo "     saluki-nxp93_flat"
  echo
  exit 1
}
if [ -z ${SIGNING_ARGS+x} ]; then
  SIGNING_ARGS=""
else
  echo "using custom signing keys: ${SIGNING_ARGS}"
fi

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
build_cmd_fw="docker run --rm -e SIGNING_ARGS=${SIGNING_ARGS} -v ${script_dir}:/px4-firmware/sources ${iname_env} ./packaging/build_px4fw.sh"
build_cmd_px4fwupdater="${script_dir}/packaging/build_px4fwupdater.sh -v ${version} -i ${dest_dir}"

# Generate build_env
if [ "${target}" != px4fwupdater ]; then
  $build_env
fi

json_output="{\"type\":\"px4-firmware\",\
              \"hw\":\"${target}\","
case $target in
  "px4fwupdater")
    $build_cmd_px4fwupdater
    ;;
  "pixhawk")
    $build_cmd_fw px4_fmu-v5x_ssrc
    cp ${script_dir}/build/px4_fmu-v5x_ssrc/px4_fmu-v5x_ssrc.px4 ${dest_dir}/px4_fmu-v5x_ssrc-${version}.px4
    ;;
  "fmu-v6xrt")
    $build_cmd_fw px4_fmu-v6xrt_bootloader
    $build_cmd_fw px4_fmu-v6xrt_ssrc
    cp ${script_dir}/build/px4_fmu-v6xrt_bootloader/px4_fmu-v6xrt_bootloader.elf ${dest_dir}/px4_fmu-v6xrt_bootloader-${version}.elf
    cp ${script_dir}/build/px4_fmu-v6xrt_ssrc/px4_fmu-v6xrt_ssrc.px4 ${dest_dir}/px4_fmu-v6xrt_ssrc-${version}.px4
    ;;
  "saluki-v1_default")
    $build_cmd_fw ssrc_saluki-v1_default
    cp ${script_dir}/build/ssrc_saluki-v1_default/ssrc_saluki-v1_default.px4 ${dest_dir}/ssrc_saluki-v1_default-${version}.px4
    ;;
  "saluki-v2_default")
    $build_cmd_fw ssrc_saluki-v2_default
    cp ${script_dir}/build/ssrc_saluki-v2_default/ssrc_saluki-v2_default.px4 ${dest_dir}/ssrc_saluki-v2_default-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-v2_default/ssrc_saluki-v2_default_kernel.elf ${dest_dir}/ssrc_saluki-v2_default_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-v2_default-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-v2_default/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-v2_amp")
    $build_cmd_fw ssrc_saluki-v2_amp
    cp ${script_dir}/build/ssrc_saluki-v2_amp/ssrc_saluki-v2_amp.bin ${dest_dir}/ssrc_saluki-v2_amp-${version}.bin
    cp ${script_dir}/build/ssrc_saluki-v2_amp/ssrc_saluki-v2_amp_kernel.elf ${dest_dir}/ssrc_saluki-v2_amp_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-v2_amp-${version}.bin\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-v2_amp/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-v2_flat")
    $build_cmd_fw ssrc_saluki-v2_flat
    cp ${script_dir}/build/ssrc_saluki-v2_flat/ssrc_saluki-v2_flat.px4 ${dest_dir}/ssrc_saluki-v2_flat-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-v2_flat/ssrc_saluki-v2_flat.elf ${dest_dir}/ssrc_saluki-v2_flat-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-v2_flat-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-v2_flat/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-v2_custom_keys")
    # on custom keys case we build _default target but SIGNING_ARGS env variable is set above in build_cmd_fw
    $build_cmd_fw ssrc_saluki-v2_default
    cp ${script_dir}/build/ssrc_saluki-v2_default/ssrc_saluki-v2_default.px4 ${dest_dir}/ssrc_saluki-v2_custom_keys-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-v2_default/ssrc_saluki-v2_default_kernel.elf ${dest_dir}/ssrc_saluki-v2_custom_keys_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-v2_custom_keys-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-v2_default/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-v3_default")
    $build_cmd_fw ssrc_saluki-v3_default
    cp ${script_dir}/build/ssrc_saluki-v3_default/ssrc_saluki-v3_default.px4 ${dest_dir}/ssrc_saluki-v3_default-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-v3_default/ssrc_saluki-v3_default_kernel.elf ${dest_dir}/ssrc_saluki-v3_default_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-v3_default-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-v3_default/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-v3_amp")
    $build_cmd_fw ssrc_saluki-v3_amp
    cp ${script_dir}/build/ssrc_saluki-v3_amp/ssrc_saluki-v3_amp.px4 ${dest_dir}/ssrc_saluki-v3_amp-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-v3_amp/ssrc_saluki-v3_amp_kernel.elf ${dest_dir}/ssrc_saluki-v3_amp_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-v3_amp-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-v3_amp/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-v3_flat")
    $build_cmd_fw ssrc_saluki-v3_flat
    cp ${script_dir}/build/ssrc_saluki-v3_flat/ssrc_saluki-v3_flat.px4 ${dest_dir}/ssrc_saluki-v3_flat-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-v3_flat/ssrc_saluki-v3_flat.elf ${dest_dir}/ssrc_saluki-v3_flat-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-v3_flat-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-v3_flat/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-v3_custom_keys")
    # on custom keys case we build _default target but SIGNING_ARGS env variable is set above in build_cmd_fw
    $build_cmd_fw ssrc_saluki-v3_default
    cp ${script_dir}/build/ssrc_saluki-v3_default/ssrc_saluki-v3_default.px4 ${dest_dir}/ssrc_saluki-v3_custom_keys-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-v3_default/ssrc_saluki-v3_default_kernel.elf ${dest_dir}/ssrc_saluki-v3_custom_keys_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-v3_custom_keys-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-v3_default/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-pi_default")
    $build_cmd_fw ssrc_saluki-pi_default
    cp ${script_dir}/build/ssrc_saluki-pi_default/ssrc_saluki-pi_default.px4 ${dest_dir}/ssrc_saluki-pi_default-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-pi_default/ssrc_saluki-pi_default_kernel.elf ${dest_dir}/ssrc_saluki-pi_default_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-pi_default-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-pi_default/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-pi_amp")
    $build_cmd_fw ssrc_saluki-pi_amp
    cp ${script_dir}/build/ssrc_saluki-pi_amp/ssrc_saluki-pi_amp.px4 ${dest_dir}/ssrc_saluki-pi_amp-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-pi_amp/ssrc_saluki-pi_amp_kernel.elf ${dest_dir}/ssrc_saluki-pi_amp_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-pi_amp-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-pi_amp/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-pi_flat")
    $build_cmd_fw ssrc_saluki-pi_flat
    cp ${script_dir}/build/ssrc_saluki-pi_flat/ssrc_saluki-pi_flat.px4 ${dest_dir}/ssrc_saluki-pi_flat-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-pi_flat/ssrc_saluki-pi_flat.elf ${dest_dir}/ssrc_saluki-pi_flat-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-pi_flat-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-pi_flat/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-pi_custom_keys")
    # on custom keys case we build _default target but SIGNING_ARGS env variable is set above in build_cmd_fw
    $build_cmd_fw ssrc_saluki-pi_default
    cp ${script_dir}/build/ssrc_saluki-pi_default/ssrc_saluki-pi_default.px4 ${dest_dir}/ssrc_saluki-pi_custom_keys-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-pi_default/ssrc_saluki-pi_default_kernel.elf ${dest_dir}/ssrc_saluki-pi_custom_keys_kernel-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-pi_custom_keys-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-pi_default/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  "saluki-nxp93_flat")
    $build_cmd_fw ssrc_saluki-nxp93_flat
    cp ${script_dir}/build/ssrc_saluki-nxp93_flat/ssrc_saluki-nxp93_flat.px4 ${dest_dir}/ssrc_saluki-nxp93_flat-${version}.px4
    cp ${script_dir}/build/ssrc_saluki-nxp93_flat/ssrc_saluki-nxp93_flat.elf ${dest_dir}/ssrc_saluki-nxp93_flat-${version}.elf
    json_output+="\"filename\":\"ssrc_saluki-nxp93_flat-${version}.px4\","
    px4_build_time=$(grep px4_build_time ${script_dir}/build/ssrc_saluki-nxp93_flat/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  *)
    usage
    ;;
esac

# save json output to target directory
echo $json_output > ${dest_dir}/ssrc_${target}-${version}.json

echo "Done"
