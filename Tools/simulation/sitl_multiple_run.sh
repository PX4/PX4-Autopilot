#!/usr/bin/env bash
# Run multiple instances of the 'px4' binary, without starting an external simulator.
# It assumes px4 is already built with the specified build target.
#
# Usage: ./Tools/simulation/sitl_multiple_run.sh [num_instances] [model] [build_target]
# Examples:
#   ./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih
#   ./Tools/simulation/sitl_multiple_run.sh 2 gazebo-classic_iris px4_sitl_default
#   ./Tools/simulation/sitl_multiple_run.sh     # defaults: 2 instances, gazebo-classic_iris, px4_sitl_default
#
# Pass 0 instances to just stop everything currently running:
#   ./Tools/simulation/sitl_multiple_run.sh 0

sitl_num=${1:-2}
sim_model=${2:-gazebo-classic_iris}
build_target=${3:-px4_sitl_default}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../"

build_path=${src_path}/build/${build_target}

# Pick px4 vs px4.exe so this script also runs from Git Bash on Windows.
px4_bin="$build_path/bin/px4"
[ -x "$px4_bin.exe" ] && px4_bin="$px4_bin.exe"

echo "killing running instances"
if command -v pkill >/dev/null 2>&1; then
	pkill -x px4 || true
fi
if command -v taskkill >/dev/null 2>&1; then
	taskkill //IM px4.exe //F >/dev/null 2>&1 || true
fi

sleep 1

export PX4_SIM_MODEL=${sim_model}

n=0
while [ $n -lt $sitl_num ]; do
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	"$px4_bin" -i $n -d "$build_path/etc" >out.log 2>err.log &
	popd &>/dev/null

	n=$(($n + 1))
done
