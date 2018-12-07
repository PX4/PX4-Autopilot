#!/bin/bash


script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
px4_dir=${script_dir}/../..

source "${px4_dir}/Tools/setup_gazebo.bash" "${px4_dir}" "${px4_dir}/build/px4_sitl_default"
${script_dir}/run_tests.py
