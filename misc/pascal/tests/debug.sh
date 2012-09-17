#!/bin/sh
############################################################################
# debug.sh
#
#   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
############################################################################
#set -x

source ../.config

if [ "${CONFIG_INSN16}" == "y" ]; then
    BINDIR=bin16
fi
if [ "${CONFIG_INSN32}" == "y" ]; then
    BINDIR=bin32
fi

# Tell them how they are supposed to use this script
function show_usage ()
{
    echo "USAGE:"
    echo "  ${0} [OPTION] <pex-file-basename>"
    echo "OPTIONS:"
    echo "  -t <strstksz>: Select string stack size"
    echo "  -h:            Show this text"
    exit 1
}

STRSTKSZ=1024
PEXFILENAME=

# Parse command line
while [ -n "$1" ]; do
    case "$1" in
	-t )
	    STRSTKSZ=$2
	    shift
	    ;;
	-h )
	    show_usage
	    ;;
	* )
	    PEXFILENAME=$1
	    ;;
    esac
    shift
done

echo "Using string stack size = ${STRSTKSZ}"
PRUN=../${BINDIR}/prun
PRUNOPTS="-d -t ${STRSTKSZ}"

if [ -z "${PEXFILENAME}" ]; then
    echo "ERROR: No file name provided"
    exit 1
fi

PEXBASENAME=`basename ${PEXFILENAME} .pas`
PEXDIRNAME=`dirname ${PEXFILENAME}`
if [ "${PEXDIRNAME}" == "." ]; then
    PEXDIRNAME=src
fi

PEXFILENAME=${PEXDIRNAME}/${PEXBASENAME}.pex
if [ ! -f "${PEXFILENAME}" ]; then
    echo "ERROR: ${PEXFILENAME} does not exist"
    exit 1
fi

if [ -f ${PEXDIRNAME}/${PEXBASENAME}.inp ] ; then
    echo "Test command line arguments:"
    echo "  \"`cat ${PEXDIRNAME}/${PEXBASENAME}.inp`\""
fi

${PRUN} ${PRUNOPTS} src/${PEXBASENAME}.pex 2>&1
