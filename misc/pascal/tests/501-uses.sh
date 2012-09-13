#!/bin/sh
############################################################################
# tests/501-uses.sh
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
PLINK=../${BINDIR}/plink
PRUN=../${BINDIR}/prun

PASOPTS=-Isrc
PRUNOPTS="-t 1024"

FILE1=src/501-unit-cosine
FILE2=src/501-unit-sine
FILE3=src/501-unit-data
FILE4=src/501-uses

FILES="${FILE1}.pas ${FILE2}.pas ${FILE3}.pas ${FILE4}.pas"
PROG=src/501-uses.pex

for file in ${FILES}; do
    echo "########${file}########";
    basefile=`basename ${file} .pas`
    ${PASCAL} ${PASOPTS} ${file} 2>&1 || rm -f src/${basefile}.o1
    if [ -f src/${basefile}.o1 ] ; then
	${POPT} src/${basefile}.o1 2>&1 || rm -f src/${basefile}.o*
    fi
    cat src/${basefile}.err | grep Line
done

if [ -f ${FILE1}.o ] ; then
    if [ -f ${FILE2}.o ] ; then
	if [ -f ${FILE3}.o ] ; then
	    if [ -f ${FILE4}.o ] ; then
		echo "########${PROG}########";
		${PLINK} ${FILE1}.o ${FILE2}.o ${FILE3}.o ${FILE4}.o ${PROG} 2>&1
		${PRUN} ${PRUNOPTS} ${PROG} 2>&1 <src/501-uses.inp
	    fi
	fi
    fi
fi
exit
