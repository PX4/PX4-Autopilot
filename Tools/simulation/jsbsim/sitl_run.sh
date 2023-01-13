#!/usr/bin/env bash


set -e

if [ "$#" -lt 5 ]; then
	echo usage: sitl_run.sh sitl_bin model world src_path build_path
	exit 1
fi

if [[ -n "$DONT_RUN" ]]; then
	echo "Not running simulation (DONT_RUN is set)."
	exit 0
fi

sitl_bin="$1"
model="$2"
world="$3"
src_path="$4"
build_path="$5"

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo model: $model
echo world: $world
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

if [ -n $FG_BINARY ]; then
    FG_BINARY=fgfs
fi

export PX4_SIM_MODEL=jsbsim_${model}
export PX4_SIM_WORLD=${world}

# This is needed for aircraft namespace mapping
# Need more architectural discussions to make this more scalable
case "$model" in
        rascal)
            MODEL_NAME="Rascal110-JSBSim"
            ;;
        malolo)
            MODEL_NAME="Malolo1"
            ;;
        quadrotor_x)
            MODEL_NAME="quadrotor_x"
            ;;
        hexarotor_x)
            MODEL_NAME="hexarotor_x"
            ;;
        *)
            echo "Unknown Model"
            exit 1

esac

export JSBSIM_AIRCRAFT_MODEL="$MODEL_NAME"

if [[ -n "$HEADLESS" ]]; then
	echo "not running flightgear gui"
else
	export FG_AIRCRAFT="${SRC_DIR}/Tools/simulation/jsbsim/jsbsim_bridge/models"

	$FG_BINARY --fdm=null \
		--native-fdm=socket,in,60,,5550,udp \
		--aircraft=$JSBSIM_AIRCRAFT_MODEL \
		--airport=${world} \
		--disable-hud \
		--disable-ai-models &> /dev/null &
	FGFS_PID=$!
fi

"${build_path}/build_jsbsim_bridge/jsbsim_bridge" ${model} -s "${src_path}/Tools/simulation/jsbsim/jsbsim_bridge/scene/${world}.xml" 2> /dev/null &
JSBSIM_PID=$!

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="\"$sitl_bin\" $no_pxh \"$build_path\"/etc"

echo SITL COMMAND: $sitl_command
eval $sitl_command

popd >/dev/null

kill $JSBSIM_PID
kill $FGFS_PID
