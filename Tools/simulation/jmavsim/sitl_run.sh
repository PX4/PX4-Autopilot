#!/usr/bin/env bash

set -e

if [ "$#" -lt 3 ]; then
	echo usage: sitl_run.sh sitl_bin debugger src_path build_path
	exit 1
fi

sitl_bin="$1"
debugger="$2"
src_path="$3"
build_path="$4"

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo debugger: $debugger
echo src_path: $src_path
echo build_path: $build_path

rootfs="$build_path/rootfs" # this is the working directory
mkdir -p "$rootfs"

# To disable user input
if [[ -n "$NO_PXH" ]]; then
	no_pxh=-d
else
	no_pxh=""
fi

jmavsim_pid=`ps aux | grep java | grep "\-jar jmavsim_run.jar" | awk '{ print $2 }'`
if [ -n "$jmavsim_pid" ]; then
	kill $jmavsim_pid
fi

export PX4_SIM_MODEL="iris"

# Start Java simulator
"$src_path"/Tools/simulation/jmavsim/jmavsim_run.sh -r 250 -l &
SIM_PID=$!

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="\"$sitl_bin\" $no_pxh \"$build_path\"/etc"

echo SITL COMMAND: $sitl_command

if [ "$debugger" == "lldb" ]; then
	eval lldb -- $sitl_command
elif [ "$debugger" == "gdb" ]; then
	eval gdb --args $sitl_command
elif [ "$debugger" == "valgrind" ]; then
	eval valgrind --track-origins=yes --leak-check=full -v $sitl_command
elif [ "$debugger" == "callgrind" ]; then
	eval valgrind --tool=callgrind -v $sitl_command
else
	eval $sitl_command
fi

popd >/dev/null

pkill -9 -P $SIM_PID
kill -9 $SIM_PID
