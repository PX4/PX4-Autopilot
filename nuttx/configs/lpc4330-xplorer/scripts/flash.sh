#!/bin/bash
####################################################################################
# flash.sh
#
#   Copyright (C) 2012 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
####################################################################################
set -x

USAGE="$0 <nuttx-path>"

# LPCXpresso 3.6 installed at /cygdrive/c/nxp/lpcxpresso_3.6"
BINDIR="/cygdrive/c/nxp/LPCXpresso_4.2.3_292/lpcxpresso/bin"

# RedSuite with LPC4330 support installed at /cygdrive/c/code_red/RedSuite_4.2.3_379 "
#BINDIR="/cygdrive/c/code_red/RedSuite_4.2.3_379/redsuite/bin"

TARGET=LPC4330

echo "############################################################################"
echo "# Assumptions:"
echo "#"
echo "#   - Windows 7"
echo "#   - Binaries installed at ${BINDIR}"
echo "#   - AXF image built with Code Red"
echo "#   - ${TARGET}"
echo "#"
echo "# You will need to edit this is any of the above are false"
echo "#"
echo "############################################################################"
echo ""

# This is the default install location for binaries on Windows (note that this
# path could change with the Code Red version number)

if [ ! -d "${BINDIR}" ]; then
	echo "Directory ${BINDIR} does not exist"
	exit 1
fi

# This is the relative path to the booLPCXpresso utility.

BOOTLPC="Scripts/bootLPCXpresso.cmd"
if [ ! -x "${BINDIR}/${BOOTLPC}" ]; then
	echo "No executable at ${BINDIR}/${BOOTLPC}"
	exit 1
fi

# bootLPCXpresso arguments

BOOTLPC_ARG=winusb      # Win7

# Use the LPC18xx/LPC43xx flash utility

FLASHUTIL="crt_emu_lpc18_43_nxp"  # for LPC18xx/LPC43xx parts

if [ ! -x "${BINDIR}/${FLASHUTIL}" ]; then
	echo "No executable file at ${BINDIR}/${FLASHUTIL}"
	exit 1
fi

# FLUSHUTIL arguements

WIRE="-wire=winusb" # for LPC-Link on Windows Vista/Windows 7)

# The nuttx directory must be provided as an argument

NUTTX=$1
if [ -z "${NUTTX}" ]; then
	echo "Missing argument"
	echo $USAGE
	exit 1
fi

if [ ! -d "${NUTTX}" ]; then
	echo "Directory ${NUTTX} does not exist"
	echo $USAGE
	exit 1
fi

# The binary to download:

if [ ! -f "${NUTTX}/nuttx.axf" ]; then
	if [ -f "${NUTTX}/nuttx" ]; then
		echo "Renaming ${NUTTX}/nuttx to ${NUTTX}/nuttx.axf"
		mv ${NUTTX}/nuttx ${NUTTX}/nuttx.axf
	fi
else
	if [ -f "${NUTTX}/nuttx" ]; then
		echo "Both ${NUTTX}/nuttx ${NUTTX}/nuttx.axf exist.."
		echo "  Deleting ${NUTTX}/nuttx.axf"
		rm -f ${NUTTX}/nuttx.axf
		echo "Renaming ${NUTTX}/nuttx to ${NUTTX}/nuttx.axf"
		mv ${NUTTX}/nuttx ${NUTTX}/nuttx.axf
	fi
fi
NUTTXPATH=`cygpath -w "${NUTTX}/nuttx.axf"`

# First of all boot the LPC-Link using the script: ${BINDIR}/${BOOTLPC}

cd ${BINDIR} || \
	{ echo "Failed to CD to ${BINDIR}"; exit 1; }
./${BOOTLPC} ${BOOTLPC_ARG} || \
	{ echo "'${BOOTLPC} ${BOOTLPC_ARG}' Failed"; }

echo ""
echo "Wait a bit"
echo "5..."
sleep 1
echo "4..."
sleep 1
echo "3..."
sleep 1
echo "2..."
sleep 1
echo "1..."
sleep 1
echo "0..."
echo ""

# Then program the FLASH

cd ${BINDIR} || \
	{ echo "Failed to CD to ${BINDIR}"; exit 1; }
./${FLASHUTIL} ${WIRE} -p${TARGET} -flash-load-exec="${NUTTXPATH}"
