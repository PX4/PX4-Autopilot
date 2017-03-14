#! /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/jMAVSim"

udp_port=14560
extra_args=
baudrate=921600
device=
ip="127.0.0.1"
while getopts ":b:d:p:qr:i:" opt; do
	case $opt in
		b)
			baudrate=$OPTARG
			;;
		d)
			device="$OPTARG"
			;;
		i)
			ip="$OPTARG"
			;;
		p)
			udp_port=$OPTARG
			;;
		q)
			extra_args="$extra_args -qgc"
			;;
		r)
			extra_args="$extra_args -r $OPTARG"
			;;
		\?)
			echo "Invalid option: -$OPTARG" >&2
			exit 1
			;;
	esac
done

if [ "$device" == "" ]; then
	device="-udp $ip:$udp_port"
else
	device="-serial $device $baudrate"
fi

ant create_run_jar copy_res
cd out/production
java -Djava.ext.dirs= -jar jmavsim_run.jar $device $extra_args
