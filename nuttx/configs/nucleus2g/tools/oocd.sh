#!/bin/sh

TOPDIR=$1
CFGFILE=$2
USAGE="$0 <TOPDIR> [-d]"

if [ -z "${TOPDIR}" ]; then
	echo "Missing argument"
	echo $USAGE
	exit 1
fi

if [ -z "${CFGFILE}" ]; then
	echo "Using olimex.cfg"
    CFGFILE=olimex.cfg
fi

OPENOCD_PATH="/cygdrive/c/OpenOCD/openocd-0.4.0/src"
OPENOCD_EXE=openocd.exe
OPENOCD_CFG="${TOPDIR}/configs/nucleus2g/tools/${CFGFILE}"
OPENOCD_ARGS="-f `cygpath -w ${OPENOCD_CFG}`"

if [ "X$2" = "X-d" ]; then
	OPENOCD_ARGS=$OPENOCD_ARGS" -d3"
	set -x
fi

if [ ! -d ${OPENOCD_PATH} ]; then
	echo "OpenOCD path does not exist: ${OPENOCD_PATH}"
	exit 1
fi
if [ ! -x ${OPENOCD_PATH}/${OPENOCD_EXE} ]; then
	echo "OpenOCD does not exist: ${OPENOCD_PATH}/${OPENOCD_EXE}"
	exit 1
fi
if [ ! -f ${OPENOCD_CFG} ]; then
	echo "OpenOCD config file does not exist: ${OPENOCD_CFG}"
	exit 1
fi

echo "Starting OpenOCD"
${OPENOCD_PATH}/${OPENOCD_EXE} ${OPENOCD_ARGS} &
echo "OpenOCD daemon started"
ps -ef | grep openocd
echo "In GDB: target remote localhost:3333"




