#!/bin/bash
# run multiple instances of the 'px4' binary, but w/o starting the simulator.
# It assumes px4 is already built, with 'make posix_sitl_default'

sitl_num=2

sim_port=15019
mav_port=15010
mav_port2=15011

mav_oport=15015
mav_oport2=15016

port_step=10

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

rc_script="posix-configs/SITL/init/ekf2/multiple_iris"
build_path=${src_path}/build_posix_sitl_default

echo "killing running instances"
pkill -x px4 || true

sleep 1

cd $build_path

user=`whoami`
n=1
while [ $n -le $sitl_num ]; do
	working_dir="instance_$n"
	if [ ! -d $working_dir ]; then
		mkdir -p "$working_dir"
		pushd "$working_dir" &>/dev/null

		# replace template config with configured ports of current instance
		cat ${src_path}/${rc_script} | sed s/_SIMPORT_/${sim_port}/ | \
			sed s/_MAVPORT_/${mav_port}/g | sed s/_MAVOPORT_/${mav_oport}/ | \
			sed s/_MAVPORT2_/${mav_port2}/ | sed s/_MAVOPORT2_/${mav_oport2}/ > rcS
		popd &>/dev/null
	fi

	pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	sudo -b -u $user ../src/firmware/posix/px4 -d "$src_path" rcS >out.log 2>err.log
	popd &>/dev/null

	n=$(($n + 1))
	sim_port=$(($sim_port + $port_step))
	mav_port=$(($mav_port + $port_step))
	mav_port2=$(($mav_port2 + $port_step))
	mav_oport=$(($mav_oport + $port_step))
	mav_oport2=$(($mav_oport2 + $port_step))
done
