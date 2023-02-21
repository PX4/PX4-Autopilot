#!/usr/bin/env bash

set -e

if [ "$#" -lt 4 ]; then
	echo usage: sitl_run.sh sitl_bin model src_path build_path
	exit 1
fi

sitl_bin="$1"
model="$2"
src_path="$3"
build_path="$4"

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo model: $model
echo src_path: $src_path
echo build_path: $build_path

rootfs="$build_path/rootfs" # this is the working directory
mkdir -p "$rootfs"

export PX4_SIM_MODEL=flightgear_${model}

echo "FG setup"
cd "${src_path}/Tools/simulation/flightgear/flightgear_bridge/"
./FG_run.py models/${model}.json 0
"${build_path}/build_flightgear_bridge/flightgear_bridge" 0 `./get_FGbridge_params.py "models/"${model}".json"` &
FG_BRIDGE_PID=$!

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="\"$sitl_bin\" \"$build_path\"/etc"

echo SITL COMMAND: $sitl_command

eval $sitl_command

popd >/dev/null

kill $FG_BRIDGE_PID
kill -9 `cat /tmp/px4fgfspid_0`
