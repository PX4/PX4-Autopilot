#!/bin/sh
#
# Wrapper to upload a PX4 firmware binary
#
TOOLS=`dirname $0`
MKFW=${TOOLS}/px_mkfw.py
UPLOAD=${TOOLS}/px_uploader.py

BINARY=nuttx.bin
PAYLOAD=nuttx.px4
PORTS="/dev/tty.usbmodemPX1,/dev/tty.usbmodemPX2,/dev/tty.usbmodemPX3,/dev/tty.usbmodemPX4,/dev/tty.usbmodem1,/dev/tty.usbmodem2,/dev/tty.usbmodem3,/dev/tty.usbmodem4"

function abort() {
	echo "ABORT: $*"
	exit 1
}

if [ ! -f ${MKFW} -o ! -f ${UPLOAD} ]; then
	abort "Missing tools ${MKFW} and/or ${UPLOAD}"
fi
if [ ! -f ${BINARY} ]; then
	abort "Missing nuttx binary in current directory."
fi

rm -f ${PAYLOAD}
${MKFW} --board_id 5 --image ${BINARY} > ${PAYLOAD}
${UPLOAD} --port ${PORTS} ${PAYLOAD}
