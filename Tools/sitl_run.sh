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
rootfs=$build_path/tmp/rootfs

if [ "$chroot" == "1" ]
then
	chroot_enabled=-c
	sudo_enabled=sudo
else
	chroot_enabled=""
	sudo_enabled=""
fi

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
	$src_path/Tools/jmavsim_run.sh -r 500 &
	SIM_PID=`echo $!`
	cd ../..
elif [ "$program" == "gazebo" ] && [ ! -n "$no_sim" ]
then
	if [ -x "$(command -v gazebo)" ]
	then
		if  [[ -z "$DONT_RUN" ]]
		then
			# Set the plugin path so Gazebo finds our model and sim
			source $src_path/Tools/setup_gazebo.bash ${src_path} ${build_path}

			gzserver --verbose ${src_path}/Tools/sitl_gazebo/worlds/${model}.world &
			SIM_PID=`echo $!`

			if [[ -n "$HEADLESS" ]]; then
				echo "not running gazebo gui"
			else
				# gzserver needs to be running to avoid a race. Since the launch
				# is putting it into the background we need to avoid it by backing off
				sleep 3
				nice -n 20 gzclient --verbose &
				GUI_PID=`echo $!`
			fi
		fi
	else
		echo "You need to have gazebo simulator installed!"
		exit 1
	fi
fi

cd $working_dir

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="$sudo_enabled $sitl_bin $no_pxh $chroot_enabled $src_path $src_path/${rcS_dir}/${model}"

echo SITL COMMAND: $sitl_command

if [[ -n "$DONT_RUN" ]]
then
    echo "Not running simulation (\$DONT_RUN is set)."
# Start Java simulator
elif [ "$debugger" == "lldb" ]
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
	$sitl_command
fi

if [[ -z "$DONT_RUN" ]]
then
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
fi
