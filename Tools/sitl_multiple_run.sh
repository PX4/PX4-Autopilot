#!/bin/bash
# run multiple instances of the 'px4' binary, but w/o starting the simulator.
# It assumes px4 is already built, with 'make posix_sitl_default'

# The simulator is expected to send to UDP port 14560+i for i in [0, N-1]
# For example jmavsim can be run like this:
#./Tools/jmavsim_run.sh -p 14561

sitl_num=2
[ -n "$1" ] && sitl_num="$1"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

build_path=${src_path}/build/posix_sitl_default

echo "killing running instances"
pkill -x px4 || true

sleep 1

export PX4_SIM_MODEL=iris

n=0
while [ $n -lt $sitl_num ]; do
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	../bin/px4 -i $n -d "$src_path/ROMFS/px4fmu_common" -s etc/init.d-posix/rcS >out.log 2>err.log &
	popd &>/dev/null

	n=$(($n + 1))
done
