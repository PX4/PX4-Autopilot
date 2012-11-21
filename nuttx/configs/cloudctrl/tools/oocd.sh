#!/bin/sh

# Get command line parameters

USAGE="USAGE: $0 [-dh] <TOPDIR>"
ADVICE="Try '$0 -h' for more information"

unset DEBUG

while [ ! -z "$1" ]; do
	case $1 in
	-d )
		set -x
		DEBUG=-d3
		;;
	-h )
		echo "$0 is a tool for generation of proper version files for the NuttX build"
		echo ""
		echo $USAGE
		echo ""
		echo "Where:"
		echo "  -d"
		echo "	  Enable script debug"
		echo "  -h"
		echo "	  show this help message and exit"
		echo "	  Use the OpenOCD 0.4.0"
		echo "  <TOPDIR>"
		echo "	  The full path to the top-level NuttX directory"
		exit 0
		;;
	* )
		break;
		;;
	esac
	shift
done

TOPDIR=$1
if [ -z "${TOPDIR}" ]; then
	echo "Missing argument"
	echo $USAGE
	echo $ADVICE
	exit 1
fi

# This script *probably* only works with the following versions of OpenOCD:

# Local search directory and configurations

OPENOCD_SEARCHDIR="${TOPDIR}/configs/shenzhou/tools"
OPENOCD_WSEARCHDIR="`cygpath -w ${OPENOCD_SEARCHDIR}`"

OPENOCD_PATH="/cygdrive/c/Program Files (x86)/OpenOCD/0.4.0/bin"
OPENOCD_EXE=openocd.exe
OPENOCD_INTERFACE="olimex-arm-usb-ocd.cfg"


OPENOCD_TARGET="stm32.cfg"
OPENOCD_ARGS="${DEBUG} -s ${OPENOCD_WSEARCHDIR} -f ${OPENOCD_INTERFACE} -f ${OPENOCD_TARGET}"

echo "Trying OpenOCD 0.4.0 path: ${OPENOCD_PATH}/${OPENOCD_EXE}"

# Verify that everything is what it claims it is and is located where it claims it is.

if [ ! -x "${OPENOCD_PATH}/${OPENOCD_EXE}" ]; then
	echo "OpenOCD executable does not exist: ${OPENOCD_PATH}/${OPENOCD_EXE}"
	exit 1
fi
if [ ! -f "${OPENOCD_SEARCHDIR}/${OPENOCD_TARGET}" ]; then
	echo "OpenOCD target config file does not exist: ${OPENOCD_SEARCHDIR}/${OPENOCD_TARGET}"
	exit 1
fi
if [ ! -f "${OPENOCD_SEARCHDIR}/${OPENOCD_INTERFACE}" ]; then
	echo "OpenOCD interface config file does not exist: ${OPENOCD_SEARCHDIR}/${OPENOCD_INTERFACE}"
	exit 1
fi

# Enable debug if so requested

if [ "X$2" = "X-d" ]; then
	OPENOCD_ARGS=$OPENOCD_ARGS" -d3"
	set -x
fi

# Okay... do it!

echo "Starting OpenOCD"
"${OPENOCD_PATH}/${OPENOCD_EXE}" ${OPENOCD_ARGS} &
echo "OpenOCD daemon started"
ps -ef | grep openocd
echo "In GDB: target remote localhost:3333"

