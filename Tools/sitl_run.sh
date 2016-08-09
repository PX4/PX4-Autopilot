#!/bin/bash

set -e

echo args: $@

sitl_bin=$1
rcS_dir=$2
debugger=$3
program=$4
model=$5
src_path=$6
build_path=$7

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo rcS_dir: $rcS_dir
echo debugger: $debugger
echo program: $program
echo model: $model
echo src_path: $src_path
echo build_path: $build_path

working_dir=`pwd`
sitl_bin=$build_path/src/firmware/posix/px4
rootfs=$build_path/tmp/rootfs

# To disable user input
if [[ -n "$NO_PXH" ]]; then
	no_pxh=-d
else
	no_pxh=""
fi

if [ "$model" == "" ] || [ "$model" == "none" ]
then
	echo "empty model, setting iris as default"
	model="iris"
fi

# check replay mode
if [ "$replay_mode" == "ekf2" ]
then
	model="iris_replay"
	# create the publisher rules
	mkdir -p $rootfs
	publisher_rules_file="$rootfs/orb_publisher.rules"
	cat <<EOF > "$publisher_rules_file"
restrict_topics: sensor_combined, vehicle_gps_position, vehicle_land_detected
module: replay
ignore_others: false
EOF
fi

if [ "$#" -lt 7 ]
then
	echo usage: sitl_run.sh rc_script rcS_dir debugger program model src_path build_path
	echo ""
	exit 1
fi

# kill process names that might stil
# be running from last time
pkill -x gazebo || true
pkill -x px4 || true
pkill -x px4_$model || true

jmavsim_pid=`ps aux | grep java | grep Simulator | cut -d" " -f1`
if [ -n "$jmavsim_pid" ]
then
	kill $jmavsim_pid
fi

cp $src_path/Tools/posix_lldbinit $working_dir/.lldbinit
cp $src_path/Tools/posix.gdbinit $working_dir/.gdbinit

SIM_PID=0

if [ "$program" == "jmavsim" ] && [ ! -n "$no_sim" ]
then
	$src_path/Tools/jmavsim_run.sh &
	SIM_PID=`echo $!`
	cd ../..
elif [ "$program" == "gazebo" ] && [ ! -n "$no_sim" ]
then
	if [ -x "$(command -v gazebo)" ]
	then
		# Set the plugin path so Gazebo finds our model and sim
		source $src_path/Tools/setup_gazebo.bash ${src_path} ${build_path}

		gzserver --verbose ${src_path}/Tools/sitl_gazebo/worlds/${model}.world &
		SIM_PID=`echo $!`

		if [[ -n "$HEADLESS" ]]; then
			echo "not running gazebo gui"
		else
			gzclient --verbose &
			GUI_PID=`echo $!`
		fi
	else
		echo "You need to have gazebo simulator installed!"
		exit 1
	fi
elif [ "$program" == "replay" ] && [ ! -n "$no_sim" ]
then
	echo "Replaying logfile: $logfile"
	# This is not a simulator, but a log file to replay

	# Check if we need to creat a param file to allow user to change parameters
	if ! [ -f "$rootfs/replay_params.txt" ]
		then
		mkdir -p $rootfs
		touch $rootfs/replay_params.txt
	fi
fi

cd $working_dir

if [ "$logfile" != "" ]
then
	cp $logfile $rootfs/replay.px4log
fi

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="$sudo_enabled $sitl_bin $no_pxh $src_path etc/init/${rc_script}"

echo SITL COMMAND: $sitl_command

# Prepend to path to prioritize PX4 commands over potentially already
# installed PX4 commands.
export PATH="$build_path/bin":$PATH

export SIM_MODEL=${model}
export SIM_PROGRAM=${program}

# Start Java simulator
if [ "$debugger" == "lldb" ]
then
	lldb -- $sitl_command
elif [ "$debugger" == "gdb" ]
then
	gdb --args $sitl_command
elif [ "$debugger" == "ddd" ]
then
	ddd --debugger gdb --args $sitl_command
elif [ "$debugger" == "valgrind" ]
then
	valgrind --track-origins=yes --leak-check=full -v $sitl_command
elif [ "$debugger" == "callgrind" ]
then
	valgrind --tool=callgrind -v $sitl_command
elif [ "$debugger" == "ide" ]
then
	echo "######################################################################"
	echo
	echo "PX4 simulator not started, use your IDE to start PX4_${model} target."
	echo "Hit enter to quit..."
	echo
	echo "######################################################################"
	read
else
	px4 etc/init/${rc_script}
fi

if [ "$program" == "jmavsim" ]
then
	pkill -9 -P $SIM_PID
	kill -9 $SIM_PID
elif [ "$program" == "gazebo" ]
then
	kill -9 $SIM_PID
	if [[ ! -n "$HEADLESS" ]]; then
		kill -9 $GUI_PID
	fi
fi
