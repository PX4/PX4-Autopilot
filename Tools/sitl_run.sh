#!/bin/bash

rc_script=$1
debugger=$2
program=$3
model=$4
build_path=$5
curr_dir=`pwd`

echo SITL ARGS
echo rc_script: $rc_script
echo debugger: $debugger
echo program: $program
echo model: $model
echo build_path: $build_path

if [ "$model" == "" ] || [ "$model" == "none" ]
then
	echo "empty model, setting iris as default"
	model="iris"
fi

if [ "$#" != 5 ]
then
	echo usage: sitl_run.sh rc_script debugger program model build_path
	echo ""
	exit 1
fi

# kill process names that might stil
# be running from last time
pkill gazebo
pkill mainapp
jmavsim_pid=`jps | grep Simulator | cut -d" " -f1`
if [ -n "$jmavsim_pid" ]
then
	kill $jmavsim_pid
fi

cp Tools/posix_lldbinit $build_path/src/firmware/posix/.lldbinit
cp Tools/posix.gdbinit $build_path/src/firmware/posix/.gdbinit

SIM_PID=0

if [ "$program" == "jmavsim" ] && [ "$no_sim" == "" ]
then
	cd Tools/jMAVSim
	ant
	java -Djava.ext.dirs= -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator -udp 127.0.0.1:14560 &
	SIM_PID=`echo $!`
elif [ "$3" == "gazebo" ] && [ "$no_sim" == "" ]
then
	if [ -x "$(command -v gazebo)" ]
	then
		# Set the plugin path so Gazebo finds our model and sim
		export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$curr_dir/Tools/sitl_gazebo/Build
		# Set the model path so Gazebo finds the airframes
		export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$curr_dir/Tools/sitl_gazebo/models
		# The next line would disable online model lookup, can be commented in, in case of unstable behaviour.
		# export GAZEBO_MODEL_DATABASE_URI=""
		export SITL_GAZEBO_PATH=$curr_dir/Tools/sitl_gazebo
		mkdir -p Tools/sitl_gazebo/Build
		cd Tools/sitl_gazebo/Build
		cmake ..
		make -j4
		gzserver ../worlds/${model}.world &
		SIM_PID=`echo $!`
		gzclient &
		GUI_PID=`echo $!`
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
	lldb -- mainapp ../../../../${rc_script}_${program}
elif [ "$debugger" == "gdb" ]
then
	gdb --args mainapp ../../../../${rc_script}_${program}
elif [ "$debugger" == "ddd" ]
then
	ddd --debugger gdb --args mainapp ../../../../${rc_script}_${program}
elif [ "$debugger" == "valgrind" ]
then
	valgrind ./mainapp ../../../../${rc_script}_${program}_${model}
else
	./mainapp ../../../../${rc_script}_${program}_${model}
fi

if [ "$program" == "jmavsim" ]
then
	kill -9 $SIM_PID
elif [ "$program" == "gazebo" ]
then
	kill -9 $SIM_PID
	kill -9 $GUI_PID
fi
