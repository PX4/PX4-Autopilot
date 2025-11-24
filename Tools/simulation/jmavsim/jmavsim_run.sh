#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(dirname $(readlink -f "$BASH_SOURCE"))

cd "$SCRIPT_DIR/jMAVSim"

port=4560
extra_args=
baudrate=921600
device=
ip="127.0.0.1"
protocol="tcp"
while getopts "b:d:ui:p:qsr:loat" opt; do
	case $opt in
		b)
			baudrate=$OPTARG
			;;
		d)
			device="$OPTARG"
			;;
		u)
			protocol="udp"
			;;
		i)
			ip="$OPTARG"
			;;
		p)
			port=$OPTARG
			;;
		q)
			extra_args="$extra_args -qgc"
			;;
		s)
			extra_args="$extra_args -sdk"
			;;
		r)
			extra_args="$extra_args -r $OPTARG"
			;;
		l)
			extra_args="$extra_args -lockstep"
			;;
		o)
			extra_args="$extra_args -disponly"
			;;
		a)
			extra_args="$extra_args -fw" # aircraft model
			;;
		t)
			extra_args="$extra_args -ts" # tailsitter model
			;;
		\?)
			echo "Invalid option: -$OPTARG" >&2
			exit 1
			;;
	esac
done

if [ "$device" == "" ]; then
	if [ "$protocol" == "tcp" ]; then
		device="-tcp $ip:$port"
	else
		device="-udp $port"
	fi
else
	device="-serial $device $baudrate"
fi

if [ "$HEADLESS" = "1" ]; then
    extra_args="$extra_args -no-gui"
fi

ant create_run_jar copy_res
cd out/production

java --add-exports java.base/java.lang=ALL-UNNAMED --add-exports java.desktop/sun.awt=ALL-UNNAMED --add-exports java.desktop/sun.java2d=ALL-UNNAMED \
	-XX:GCTimeRatio=20 -Djava.ext.dirs= -jar jmavsim_run.jar $device $extra_args
