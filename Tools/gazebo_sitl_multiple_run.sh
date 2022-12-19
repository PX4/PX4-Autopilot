#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default gazebo'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./Tools/gazebo_sitl_multiple_run.sh -n 10 -m iris

SUPPORTED_MODELS=("iris" "plane" "standard_vtol" "rover" "r1_rover" "typhoon_h480", "shieldai_nova2", "skydio_x2d")

function cleanup() {
	pkill -x px4
	pkill gzserver

	if [[ -n "$HEADLESS" ]]; then
		exit
	else
		pkill gzclient
	fi
}

function spawn_model() {
	MODEL=$1
	N=$2 #Instance Number
	X=$3
	Y=$4
	X=${X:=0.0}
	Y=${Y:=$((3*${N}))}

	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently vehicle model '$MODEL' is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi

	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $N in $(pwd)"

	if [[ -n "${PX4_VIDEO_HOST_IP}" ]]; then
		export PX4_VIDEO_HOST_IP=${PX4_VIDEO_HOST_IP%.*}.$((7+$N))
		echo "PX4_VIDEO_HOST_IP '$PX4_VIDEO_HOST_IP'"
	fi

	if [[ -n "${PX4_SIM_REMOTE_HOST}" ]]; then
		export PX4_SIM_REMOTE_HOST=${PX4_SIM_REMOTE_HOST%.*}.$((7+$N))
		echo "PX4_SIM_REMOTE_HOST '$PX4_SIM_REMOTE_HOST'"
	fi

	../bin/px4 -i $N -d "$build_path/etc" -w sitl_${MODEL}_${N} -s etc/init.d-posix/rcS >out.log 2>err.log &
	python3 ${src_path}/Tools/sitl_gazebo/scripts/jinja_gen.py ${src_path}/Tools/sitl_gazebo/models/${MODEL}/${MODEL}.sdf.jinja ${src_path}/Tools/sitl_gazebo --mavlink_tcp_port $((4560+${N})) --mavlink_udp_port $((14560+${N})) --mavlink_id $((1+${N})) --gst_udp_host 172.5.0.$((7+${N})) --gst_udp_port $((5600)) --video_uri $((5600+${N})) --mavlink_cam_udp_port $((14530+${N})) --udp_onboard_gimbal_port_local $((13030+${N})) --output-file /tmp/${MODEL}_${N}.sdf --vehicle_id ${N}

	echo "Spawning ${MODEL}_${N} at ${X} ${Y}"

	gz model --spawn-file=/tmp/${MODEL}_${N}.sdf --model-name=${MODEL}_${N} -x ${X} -y ${Y} -z 0.83

	popd &>/dev/null

}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-w <world>] [-s <script>] [-c <custom_models>]"
	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,plane:2"
	echo "-c flag is used to allow custom models e.g. $0 -s \"my_plane my_plane_2\""
	exit 1
fi

while getopts n:m:w:s:t:c: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SCRIPT=${OPTARG};;
		t) TARGET=${OPTARG};;
		c) CUSTOM_MODELS=${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
world=${WORLD:=empty}
target=${TARGET:=px4_sitl_default}
vehicle_model=${VEHICLE_MODEL:="iris"}
export PX4_SIM_MODEL=${vehicle_model}

SUPPORTED_MODELS+=(${CUSTOM_MODELS})

echo ${SCRIPT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

build_path=${src_path}/build/${target}
mavlink_udp_port=14560
mavlink_tcp_port=4560

echo "killing running instances"
pkill -x px4 || true

sleep 1

source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}

# To use gazebo_ros ROS2 plugins
if [[ -n "$ROS_VERSION" ]] && [ "$ROS_VERSION" == "2" ]; then
	ros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
else
	ros_args=""
fi

echo "Starting gazebo"
gzserver ${src_path}/Tools/sitl_gazebo/worlds/${world}.world --verbose $ros_args &
sleep 5

n=0
if [ -z ${SCRIPT} ]; then
	if [ $num_vehicles -gt 255 ]
	then
		echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
		exit 1
	fi

	while [ $n -lt $num_vehicles ]; do
		spawn_model ${vehicle_model} $n
		n=$(($n + 1))
	done
else
	IFS=,
	for target in ${SCRIPT}; do
		target="$(echo "$target" | tr -d ' ')" #Remove spaces
		target_vehicle=$(echo $target | cut -f1 -d:)
		target_number=$(echo $target | cut -f2 -d:)
		target_x=$(echo $target | cut -f3 -d:)
		target_y=$(echo $target | cut -f4 -d:)

		if [ $n -gt 255 ]
		then
			echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
			exit 1
		fi

		m=0
		while [ $m -lt ${target_number} ]; do
			export PX4_SIM_MODEL=${target_vehicle}
			spawn_model ${target_vehicle}${LABEL} $n $target_x $target_y
			m=$(($m + 1))
			n=$(($n + 1))
		done
	done
fi

if [[ -n "$HEADLESS" ]]; then
	trap "cleanup" SIGINT SIGTERM

	while :
	do

		  sleep 5
	done
else
	trap "cleanup" SIGINT SIGTERM EXIT

	echo "Starting gazebo client"
	gzclient
fi
