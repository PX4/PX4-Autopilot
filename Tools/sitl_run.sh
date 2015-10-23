#!/bin/bash

rc_script=$1
debugger=$2
program=$3
build_path=$4

if [ "$#" != 4 ]
then
	echo usage: sitl_run.sh rc_script debugger program build_path
	echo args
	echo rc_script: $rc_script
	echo debugger: $debugger
	echo program: $program
	echo build_path: $buid_path
	exit 1
fi

cp Tools/posix_lldbinit $build_path/src/firmware/posix/.lldbinit
cp Tools/posix.gdbinit $build_path/src/firmware/posix/.gdbinit
if [ "$program" == "jmavsim" ]
then
	cd Tools/jMAVSim
	ant
	java -Djava.ext.dirs= -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator -udp 127.0.0.1:14560 &
	cd ../..
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
