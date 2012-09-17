#!/bin/sh

# Get command line parameters

USAGE="USAGE: $0 [-dhjo14] <TOPDIR>"
ADVICE="Try '$0 -h' for more information"

INTERFACE=Olimex
OPENOCD=0.1.0

while [ ! -z "$1" ]; do
	case $1 in
	-d )
		set -x
		;;
	-j )
		INTERFACE=Jlink
		;;
	-o )
		INTERFACE=Olimex
		;;
	-1 )
		OPENOCD=0.1.0
		;;
	-4 )
		OPENOCD=0.4.0
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
		echo "  -j"
		echo "	  Use the Segger J-link interface"
		echo "  -o"
		echo "	  Use the Olimex ARM USB OCD interface (Default)"
		echo "  -1"
		echo "	  Use the Olimex GCCFD OpenOCD 0.1.0 (Default)"
		echo "  -4"
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

###############################################################################
# OpenOCD 0.4.0 ###############################################################
###############################################################################
# This script *probably* only works with the following versions of OpenOCD:

if [ "X${OPENOCD}" = "X0.4.0" ]; then

	# Local search directory and configurations

	OPENOCD_SEARCHDIR="${TOPDIR}/configs/ea3152/tools"
	OPENOCD_WSEARCHDIR="`cygpath -w ${OPENOCD_SEARCHDIR}`"

	if [ "X${INTERFACE}" = "XJlink" ]; then
		OPENOCD_PATH="/cygdrive/c/Program Files (x86)/OpenOCD/0.4.0/bin"
		OPENOCD_EXE=openocd.exe
		OPENOCD_INTERFACE="jlink.cfg"
	else
		OPENOCD_PATH="/cygdrive/c/OpenOCD/openocd-0.4.0/src"
		OPENOCD_EXE=openocd.exe
		OPENOCD_INTERFACE="olimex-arm-usb-ocd.cfg"
	fi

	OPENOCD_TARGET="lpc3152.cfg"
	OPENOCD_ARGS="-s ${OPENOCD_WSEARCHDIR} -f ${OPENOCD_INTERFACE} -f ${OPENOCD_TARGET}"

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

###############################################################################

###############################################################################
# Older OpenOCD that Shipped with the ARM-OCD JTAG ############################
###############################################################################

else
	if [ "X${OPENOCD}" = "X0.1.0" ]; then

		OPENOCD_PATH="/cygdrive/c/gccfd/openocd/bin"
		OPENOCD_EXE=openocd-ftd2xx.exe

		echo "Trying GCCFD OpenOCD 0.1.0 path: ${OPENOCD_PATH}/${OPENOCD_EXE}"

		# Local search directory and configurations

		if [ "X${INTERFACE}" = "XJlink" ]; then
			echo "The Olimex OpenOCD doesn't support J-Link"
			exit 1
		fi

		OPENOCD_CFG="${TOPDIR}/configs/ea3152/tools/armusbocd.cfg"
		OPENOCD_ARGS="-f `cygpath -w ${OPENOCD_CFG}`"

		# Verify that everything is what it claims it is and is located where it claims it is.

		if [ ! -f ${OPENOCD_CFG} ]; then
			echo "OpenOCD config file does not exist: ${OPENOCD_CFG}"
			exit 1
		fi
		if [ ! -x "${OPENOCD_PATH}/${OPENOCD_EXE}" ]; then
			echo "OpenOCD executable does not exist: ${OPENOCD_PATH}/${OPENOCD_EXE}"
			exit 1
		fi
		if [ ! -d "${OPENOCD_PATH}" ]; then
			echo "OpenOCD path does not exist: ${OPENOCD_PATH}"
			exit 1
		fi
		if [ ! -f ${OPENOCD_CFG} ]; then
			echo "OpenOCD config file does not exist: ${OPENOCD_CFG}"
			exit 1
		fi
	else
		echo "Unsupported OpenOCD version"
		echo $ADVICE
		exit 1
	fi
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
