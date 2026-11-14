#!/usr/bin/env bash
# Run multiple instances of the 'px4' binary, without starting an external simulator.
# It assumes px4 is already built with the specified build target.
#
# Usage: ./Tools/simulation/sitl_multiple_run.sh [num_instances] [model] [build_target]
# Examples:
#   ./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih
#   ./Tools/simulation/sitl_multiple_run.sh 2 gazebo-classic_iris px4_sitl_default
#   ./Tools/simulation/sitl_multiple_run.sh     # defaults: 2 instances, gazebo-classic_iris, px4_sitl_default

sitl_num=${1:-2}
sim_model=${2:-gazebo-classic_iris}
build_target=${3:-px4_sitl_default}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$(cd "$SCRIPT_DIR/../.." && pwd)"

build_path="$src_path/build/$build_target"
px4_bin="$build_path/bin/px4"
etc_dir="$build_path/etc"

if ! [[ "$sitl_num" =~ ^[1-9][0-9]*$ ]]; then
	echo "ERROR: num_instances must be a positive integer, got '$sitl_num'" >&2
	exit 2
fi

if [ ! -x "$px4_bin" ]; then
	echo "ERROR: PX4 binary is missing or not executable: $px4_bin" >&2
	echo "Build target '$build_target' before running this script." >&2
	exit 1
fi

if [ ! -d "$etc_dir" ]; then
	echo "ERROR: PX4 runtime directory is missing: $etc_dir" >&2
	echo "Rebuild target '$build_target' before running this script." >&2
	exit 1
fi

echo "killing running instances"
pkill -x px4 || true

sleep 1

export PX4_SIM_MODEL="$sim_model"

n=0
while [ "$n" -lt "$sitl_num" ]; do
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	"$px4_bin" -i "$n" -d "$etc_dir" >out.log 2>err.log &
	popd &>/dev/null

	n=$((n + 1))
done
