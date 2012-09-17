#!/bin/bash
############################################################################
# examples/pashello/mkhello.sh
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

BINDIR=$1
WD=`pwd`

PASCAL=${BINDIR}/pascal
POPT=${BINDIR}/popt
PLINK=${BINDIR}/plink
PRUN=${BINDIR}/prun

PASFILENAME=hello.pas
OUFILE=hello.h
STRSTKSZ=1024

function sanity_check ()
{
	if [ ! -f "${WD}/${PASFILENAME}" ]; then
		echo "ERROR: Source ${PASFILENAME} does not exist in this directory"
		exit 1
	fi
	if [ -z "${BINDIR}" ]; then
		echo "ERROR: Path to the pascal bin/ directory not provided"
		exit 1
	fi
	if [ ! -d "${BINDIR}" ]; then
		echo "ERROR: Tool ${BINDIR} does not exist"
		exit 1
	fi
	if [ ! -x "${PASCAL}" ]; then
		echo "ERROR: Executable ${PASCAL} does not exist"
		exit 1
	fi
	if [ ! -x "${POPT}" ]; then
		echo "ERROR: Executable ${POPT} does not exist"
		exit 1
	fi
	if [ ! -x "${PLINK}" ]; then
		echo "ERROR: Executable ${PLINK} does not exist"
		exit 1
	fi
	if [ ! -x "${PRUN}" ]; then
		echo "ERROR: Executable ${PRUN} does not exist"
		exit 1
	fi
}

function compile_hello ()
{
	PASOPTS=
	${PASCAL} ${PASOPTS} ${PASFILENAME} 2>&1 || rm -f hello.o1
	if [ -f hello.err ] ; then
	    cat hello.err | grep Line
	fi
	if [ ! -f hello.o1 ] ; then
		echo "Compilation failed"
	else
		POPTOPTS=
		${POPT} ${POPTOPTS} hello.o1 2>&1
		${PLINK} hello.o hello.pex 2>&1
	fi
}

function test_program ()
{
    if [ "${CONFIG_REGM}" == "y" ]; then
	echo "Don't know how to run REGM programs yet"
    else
	echo "Using string stack size = ${STRSTKSZ}"
	PRUNOPTS="-t ${STRSTKSZ}"

	if [ ! -f hello.pex ]; then
	    echo "No p-code executable"
	else
	    if [ -f hello.inp ] ; then
		${PRUN} ${PRUNOPTS} hello.pex 2>&1 <hello.inp
	    else
		${PRUN} ${PRUNOPTS} hello.pex 2>&1
	    fi
	fi
    fi
}

function test_hello ()
{
	echo "Using string stack size = ${STRSTKSZ}"
	PRUNOPTS="-t ${STRSTKSZ}"

	if [ ! -f hello.pex ]; then
		echo "No p-code executable"
		exit 1
	else
		${PRUN} ${PRUNOPTS} hello.pex
	fi
}

function make_include ()
{
	xxd -i hello.pex >hello.h
}

sanity_check
compile_hello
rm *.o *.o1 *.lst *.err
test_hello
make_include

