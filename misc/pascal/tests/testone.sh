#!/bin/sh
############################################################################
# testone.sh
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

PASCAL=../${BINDIR}/pascal
POPT=../${BINDIR}/popt
REGM=../${BINDIR}/regm
PLINK=../${BINDIR}/plink
PRUN=../${BINDIR}/prun

# Tell them how they are supposed to use this script

function show_usage ()
{
    echo "USAGE:"
    echo "  ${0} [OPTION] <pas-file-basename>"
    echo "OPTIONS:"
    echo "  -t <strstksz>: Select string stack size"
    echo "  -h:            Show this text"
    exit 1
}

# Get the source file name and path

function get_sourcename ()
{
    PASBASENAME=`basename ${PASFILENAME} .pas`
    PASDIRNAME=`dirname ${PASFILENAME}`
    if [ "${PASDIRNAME}" == "." ]; then
	PASDIRNAME=src
    fi

    PASFILENAME=${PASDIRNAME}/${PASBASENAME}.pas
    if [ ! -f "${PASFILENAME}" ]; then
	echo "ERROR: ${PASFILENAME} does not exist"
	exit 1
    fi
}

# Compile source file

function compile_source ()
{
    if [ ! -f ${PASFILENAME} ]; then
	echo "No source file"
    else
	PASOPTS=-Isrc
	${PASCAL} ${PASOPTS} ${PASFILENAME} 2>&1 || rm -f src/${PASBASENAME}.o1
	if [ -f src/${PASBASENAME}.err ] ; then
	    cat src/${PASBASENAME}.err | grep Line
	fi
	if [ ! -f src/${PASBASENAME}.o1 ] ; then
	    echo "Compilation failed"
	else

	    if [ "${CONFIG_REGM}" == "y" ]; then
		POPTOPTS=-r
		${POPT} ${POPTOPTS} src/${PASBASENAME}.o1 2>&1
		${REGM} src/${PASBASENAME}.o1 2>&1
	    else
		POPTOPTS=
		${POPT} ${POPTOPTS} src/${PASBASENAME}.o1 2>&1
		${PLINK} src/${PASBASENAME}.o src/${PASBASENAME}.pex 2>&1
	    fi
	fi
    fi
}

# Run test

function test_program ()
{
    if [ "${CONFIG_REGM}" == "y" ]; then
	echo "Don't know how to run REGM programs yet"
    else
	echo "Using string stack size = ${STRSTKSZ}"
	PRUNOPTS="-t ${STRSTKSZ}"

	if [ ! -f src/${PASBASENAME}.pex ]; then
	    echo "No p-code executable"
	else
	    if [ -f src/${PASBASENAME}.inp ] ; then
		${PRUN} ${PRUNOPTS} src/${PASBASENAME}.pex 2>&1 <src/${PASBASENAME}.inp
	    else
		${PRUN} ${PRUNOPTS} src/${PASBASENAME}.pex 2>&1
	    fi
	fi
    fi
}


# Parse command line

STRSTKSZ=1024
PASFILENAME=

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
	    PASFILENAME=$1
	    ;;
    esac
    shift
done

if [ -z "${PASFILENAME}" ]; then
    echo "ERROR: No file name provided"
    show_usage
    exit 1
fi

# Get the source file name and path

get_sourcename
compile_source
test_program

