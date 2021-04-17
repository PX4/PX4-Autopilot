#!/bin/bash
# Run HITL simulation with PX4

function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-s <simulation>] [-m <vehicle_model>] [-w <world>] [-d <device>] [-b <baudrate>]"
	echo "  This script allows you to run HITL simulation with PX4"
	exit 1
fi

while getopts m:w:s:d:b:t:l: option
do
	case "${option}"
	in
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SIMULATION=${OPTARG};;
		d) DEVICE=${OPTARG};;
		b) BAUDRATE=${OPTARG};;
		l) LABEL=_${OPTARG};;
	esac
done

world=${WORLD:=empty}
target=${TARGET:=px4_sitl_default}
model=${VEHICLE_MODEL:="iris"}
simulation=${SIMULATION:="gazebo"}
device=${DEVICE:="/dev/ttyACM0"}
baudrate=${BAUDRATE:="921600"}
export PX4_SIM_MODEL=${model}${LABEL}

echo SIMULATION: ${simulation}
echo MODEL: ${PX4_SIM_MODEL}

echo ${SCRIPT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

sleep 1

SIM_PID=0

if [ "$simulation" == "jmavsim" ] && [ ! -n "$no_sim" ]; then
	# Start Java simulator
	"$src_path"/Tools/jmavsim_run.sh -q -s -d ${device} -b ${baudrate} -r 250 &
	SIM_PID=$!
elif [ "$simulation" == "gazebo" ] && [ ! -n "$no_sim" ]; then
	if [ -x "$(command -v gazebo)" ]; then
		# Set the plugin path so Gazebo finds our model and sim
		source "$src_path/Tools/setup_gazebo.bash" "${src_path}" "${build_path}"
		output_sdf=/tmp/${model}_hitl.sdf
		python3 ${src_path}/Tools/sitl_gazebo/scripts/jinja_gen.py ${src_path}/Tools/sitl_gazebo/models/${model}/${model}.sdf.jinja ${src_path}/Tools/sitl_gazebo --serial_enabled 1  --hil_mode 1 --serial_device ${device} --serial_baudrate ${baudrate} --mavlink_id 1 --output-file ${output_sdf}

		if [ -z $PX4_SITL_WORLD ]; then
			#Spawn predefined world
			if [ "$world" == "none" ]; then
				if [ -f ${src_path}/Tools/sitl_gazebo/worlds/${model}.world ]; then
					echo "empty world, default world ${model}.world for model found"
					world_path="${src_path}/Tools/sitl_gazebo/worlds/${model}.world"
				else
					echo "empty world, setting empty.world as default"
					world_path="${src_path}/Tools/sitl_gazebo/worlds/empty.world"
				fi
			else
				#Spawn empty world if world with model name doesn't exist
				world_path="${src_path}/Tools/sitl_gazebo/worlds/${world}.world"
			fi
		else
			if [ -f ${src_path}/Tools/sitl_gazebo/worlds/${PX4_SITL_WORLD}.world ]; then
				# Spawn world by name if exists in the worlds directory from environment variable
				world_path="${src_path}/Tools/sitl_gazebo/worlds/${PX4_SITL_WORLD}.world"
			else
				# Spawn world from environment variable with absolute path
				world_path="$PX4_SITL_WORLD"
			fi
		fi
		gzserver $verbose $world_path &
		SIM_PID=$!

		while gz model --verbose --spawn-file="${output_sdf}" --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
			echo "gzserver not ready yet, trying again!"
			sleep 1
		done

		if [[ -n "$HEADLESS" ]]; then
			echo "not running gazebo gui"
		else
			# gzserver needs to be running to avoid a race. Since the launch
			# is putting it into the background we need to avoid it by backing off
			sleep 3
			nice -n 20 gzclient --verbose $follow_mode
			GUI_PID=$!
		fi
	else
		echo "You need to have gazebo simulator installed!"
		exit 1
	fi
elif [ "$simulation" == "jsbsim" ] && [ -z "$no_sim" ]; then
	source "$src_path/Tools/setup_jsbsim.bash" "${src_path}" "${build_path}" ${model}
	if [[ -n "$HEADLESS" ]]; then
		echo "not running flightgear gui"
	else
		fgfs --fdm=null \
			--native-fdm=socket,in,60,,5550,udp \
			--aircraft=$JSBSIM_AIRCRAFT_MODEL \
			--airport=${world} \
			--disable-hud \
			--disable-ai-models &> /dev/null &
		FGFS_PID=$!
	fi
	"${build_path}/build_jsbsim_bridge/jsbsim_bridge" ${model} -d ${device} -b ${baudrate} -s "${src_path}/Tools/jsbsim_bridge/scene/${world}.xml"
	SIM_PID=$!
fi


trap "cleanup" SIGINT SIGTERM EXIT
