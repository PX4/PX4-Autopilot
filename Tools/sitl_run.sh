#!/bin/bash

rc_script=$1
debugger=$2
program=$3
build_path=$4
curr_dir=`pwd`

echo SITL ARGS
echo rc_script: $rc_script
echo debugger: $debugger
echo program: $program
echo build_path: $buid_path

if [ "$#" != 4 ]
then
	echo usage: sitl_run.sh rc_script debugger program build_path
	exit 1
fi

# kill process names that might stil
# be running from last time
pkill mainapp
jmavsim_pid=`jps | grep Simulator | cut -d" " -f1`
if [ -n "$jmavsim_pid" ]
then
	kill $jmavsim_pid
fi

cp Tools/posix_lldbinit $build_path/src/firmware/posix/.lldbinit
cp Tools/posix.gdbinit $build_path/src/firmware/posix/.gdbinit

SIM_PID=0

if [ "$program" == "jmavsim" ]
then
	cd Tools/jMAVSim
	ant
	java -Djava.ext.dirs= -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator -udp 127.0.0.1:14560 &
	SIM_PID=echo $!
elif [ "$3" == "gazebo" ]
then
	if [ -x "$(command -v gazebo)" ]
	then
		# Set the plugin path so Gazebo finds our model and sim
		GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$curr_dir/Tools/sitl_gazebo/Build
		# Set the model path so Gazebo finds the airframes
		GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$curr_dir/Tools/sitl_gazebo/models
		# Disable online model lookup since this is quite experimental and unstable
		GAZEBO_MODEL_DATABASE_URI=""
		mkdir -p Tools/sitl_gazebo/Build
		cd Tools/sitl_gazebo/Build
		cmake ..
		make -j4
		gazebo ../worlds/iris.world &
		SIM_PID=`echo $!`
	else
		echo "You need to have gazebo simulator installed!"
		exit 1
	fi
fi
cd $build_path/src/firmware/posix
mkdir -p rootfs/fs/microsd
mkdir -p rootfs/eeprom
touch rootfs/eeprom/parameters
# Start Java simulator
if [ "$debugger" == "lldb" ]
then
	lldb -- mainapp ../../../../$rc_script
elif [ "$debugger" == "gdb" ]
then
	gdb --args mainapp ../../../../$rc_script
else
	./mainapp ../../../../$rc_script
fi

if [ "$3" == "jmavsim" ]
then
	kill -9 $SIM_PID
elif [ "$3" == "gazebo" ]
then
	kill -9 $SIM_PID
fi
