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
  echo "     saluki-v2_custom_keys"
  echo "     saluki-pi_default"
  echo "     saluki-pi_amp"
  echo "     saluki-pi_flat"
  echo "     saluki-pi_custom_keys"
  echo "     saluki-v3_default"
  echo "     saluki-v3_amp"
  echo "     saluki-v3_flat"
  echo "     saluki-v3_custom_keys"
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
  px4fwupdater)
    $build_cmd_px4fwupdater
    ;;
  pixhawk)
    $build_cmd_fw px4_fmu-v5x_ssrc
    cp ${script_dir}/build/px4_fmu-v5x_ssrc/px4_fmu-v5x_ssrc.px4 ${dest_dir}/px4_fmu-v5x_ssrc-${version}.px4
    ;;
  fmu-v6xrt)
    $build_cmd_fw px4_fmu-v6xrt_bootloader
    $build_cmd_fw px4_fmu-v6xrt_ssrc
    cp ${script_dir}/build/px4_fmu-v6xrt_bootloader/px4_fmu-v6xrt_bootloader.elf ${dest_dir}/px4_fmu-v6xrt_bootloader-${version}.elf
    cp ${script_dir}/build/px4_fmu-v6xrt_ssrc/px4_fmu-v6xrt_ssrc.px4 ${dest_dir}/px4_fmu-v6xrt_ssrc-${version}.px4
    ;;
  # on custom keys case we build _default target but SIGNING_ARGS env variable is set above in build_cmd_fw
  *_custom_keys)
    #set build target to match the output name of the targe
    build_target="ssrc_${target}"
    # as the targets has to be built with default names, we need to have separate env target name for build scripts
    build_target_env=$(echo ${build_target}|sed 's/custom_keys/default/g')

    $build_cmd_fw ${build_target_env}
    cp ${script_dir}/build/${build_target_env}/${build_target_env}.px4 ${dest_dir}/${build_target}-${version}.px4
    cp ${script_dir}/build/${build_target_env}/${build_target_env}_kernel.elf ${dest_dir}/${build_target}_kernel-${version}.elf
    json_output+="\"filename\":\"${build_target}-${version}.px4\","
    px4_build_time=$(grep PX4_BUILD_TIME ${script_dir}/build/${build_target_env}/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  # handle all normal ssrc targets
  saluki-*)
    build_target="ssrc_${target}"
    $build_cmd_fw ${build_target}

    elf_target=${build_target}_kernel.elf
    # in flat builds kernel.elf has a different name
    if [[ ${build_target} == *flat ]]; then
      elf_target=${build_target}.elf
    fi

    cp ${script_dir}/build/${build_target}/${build_target}.px4 ${dest_dir}/${build_target}-${version}.px4
    cp ${script_dir}/build/${build_target}/${elf_target} ${dest_dir}/${build_target}_kernel-${version}.elf
    json_output+="\"filename\":\"${build_target}-${version}.px4\","
    px4_build_time=$(grep PX4_BUILD_TIME ${script_dir}/build/${build_target}/src/lib/version/build_git_version.h|awk '{print $3}')
    json_output+="\"px4_build_time\":\"${px4_build_time}\"}"
    ;;
  *)
    usage
    ;;
esac

# save json output to target directory
echo $json_output > ${dest_dir}/ssrc_${target}-${version}.json

echo "Done"
