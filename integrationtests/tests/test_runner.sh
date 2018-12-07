#!/bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
px4_dir=$( dirname $( dirname ${script_dir} ) )
rootfs_test=${px4_dir}/build/px4_sitl_default/tmp/rootfs_test
mkdir -p ${rootfs_test}
pushd ${rootfs_test} > /dev/null
source "${px4_dir}/Tools/setup_gazebo.bash" "${px4_dir}" "${px4_dir}/build/px4_sitl_default"
${script_dir}/run_tests.py ${px4_dir}
popd > /dev/null
