#!/bin/bash

set -e

maneuver=$1 # The name of the maneuver that should be started is passed as the first argument
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
px4_dir=$( dirname $( dirname ${script_dir} ) )
rootfs_test=${px4_dir}/build/px4_sitl_default/tmp/rootfs_test
mkdir -p ${rootfs_test}
rm -rf ${rootfs_test}/log/*  # delete the content of the log folder
pushd ${rootfs_test} > /dev/null
source "${px4_dir}/Tools/setup_gazebo.bash" "${px4_dir}" "${px4_dir}/build/px4_sitl_default"
${script_dir}/run_tests.py ${px4_dir} ${maneuver}
popd > /dev/null
