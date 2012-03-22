#!/bin/bash
#################################################################################
# NxWidgets/tools/addobjs.sh
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
# 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
#    me be used to endorse or promote products derived from this software
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
#################################################################################

# set -x

# Get input parameters

usage="Usage: $0 [OPTIONS] <lib-path> <obj-dir>"
advice="Try '$0 -h' for more information"

unset prefix
wintool=n

while [ ! -z "$1" ]; do
	case $1 in
	-d )
		set -x
		;;
	-p )
		shift
		prefix=$1
		;;
	-w )
		wintool=y
		;;
	-h )
		echo "$0 will add all object (.o) files in directory to an archive."
		echo ""
		echo $usage
		echo ""
		echo "Where:"
		echo "  <lib-path> is the full, absolute path to the library to use"
		echo "  <obj-dir> is full path to the directory containing the object files to be added"
		echo "OPTIONS include:"
		echo "  -p Prefix to use.  For example, to use arm-elf-ar, add '-p arm-elf-'"
		echo "  -w Use Windows style paths insted of POSIX paths"
		echo "  -d Enable script debug"
		echo "  -h Show this usage information"
		exit 0
		;;
	* )
		break;
	;;
	esac
	shift
done

libpath=$1
objdir=$2
archiver="${prefix}"ar

# Verify input parameters

if [ -z "${libpath}" ]; then
	echo "Missing required arguments"
	echo ""
	echo $usage
	echo $advice
	exit 1
fi

if [ -z "${objdir}" ]; then
	echo "Missing required argument <obj-dir>"
	echo ""
	echo $usage
	echo $advice
	exit 1
fi

if [ ! -w ${libpath} ]; then
	if [ -e ${libpath} ]; then
		echo "${libpath} exists but is not a write-able file"
		echo $advice
	else
		echo "${libpath} does not exist"
		echo $advice
	fi
	exit 1
fi

if [ ! -d ${objdir} ]; then
	if [ -e ${objdir} ]; then
		echo "${objdir} exists but is not a directory"
		echo $advice
	else
		echo "${objdir} does not exist"
		echo $advice
	fi
	exit 1
fi

# Add each object file in <obj-dir> to the archive at <lib-path>

for obj in `ls "${objdir}"/*.o`; do
	name=`basename "${obj}"`
	if [ "X${wintool}" = "Xy" ]; then
		objpath=`cygpath -w "${obj}"`
	else
		objpath=${obj}
	fi
	echo "AR:  ${name}"
	${archiver} rcs ${libpath} ${objpath} || \
		{ echo "Failed to archive the object file:"; \
		  echo "  Archive: ${libpath}"; \
		  echo "   Object: ${obj}"; \
		  exit 1; \
		}
done