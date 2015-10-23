#!/bin/bash
cp Tools/posix_lldbinit build_posix_sitl_simple/src/firmware/posix/.lldbinit
cp Tools/posix.gdbinit build_posix_sitl_simple/src/firmware/posix/.gdbinit

SIM_PID=0

if [ "$3" == "jmavsim" ]
then
	cd Tools/jMAVSim
	ant
	java -Djava.ext.dirs= -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator -udp 127.0.0.1:14560 &
	SIM_PID=echo $!
	cd ../..
elif [ "$3" == "gazebo" ]
then
	if [ -x "$(command -v gazebo)" ]
	then
		gazebo ${SITL_GAZEBO_PATH}/worlds/iris.world &
		SIM_PID=echo $!
	else
		echo "You need to have gazebo simulator installed!"
		exit 1
	fi
fi
cd build_posix_sitl_simple/src/firmware/posix
mkdir -p rootfs/fs/microsd
mkdir -p rootfs/eeprom
touch rootfs/eeprom/parameters
# Start Java simulator
if [ "$2" == "lldb" ]
then
	lldb -- mainapp ../../../../$1
elif [ "$2" == "gdb" ]
then
	gdb --args mainapp ../../../../$1
else
	./mainapp ../../../../$1
fi

if [ "$3" == "jmavsim" ]
then
	kill -9 $SIM_PID
elif [ "$3" == "gazebo" ]
then
	kill -9 $SIM_PID
fi
