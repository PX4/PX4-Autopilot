#!/bin/sh
############################################################################
# testall.sh
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

TSTLST="\
./501-uses.sh\
"

# Parse command line
DOLIST=A
if [ -n "$1" ]; then
    case "$1" in
	[0,1,2,5,8,9,A] )
	    DOLIST=${1}
	    ;;
	a )
	    DOLIST=A
	    ;;
	* )
	    echo ""; echo "Unrecognized filter option"
	    show_usage
	;;
    esac
fi

# Tell them how they are supposed to use this script
function show_usage ()
{
    echo "USAGE:"
    echo "  ${0} [FILTER]"
    echo "WHERE:"
    echo "  FILTER=0: Execute on basic functionaly tests"
    echo "  FILTER=1: Execute only math and runtime libraries"
    echo "  FILTER=2: Execute only string tests"
    echo "  FILTER=5: Execute only multi-file tests"
    echo "  FILTER=8: Execute only Erie pascal programs"
    echo "  FILTER=9: Execute only miscellaneous large programs"
    echo "  FILTER=A: Execute all tests"
    echo "  Default:  Execute all tests"
    exit 1
}

# Check if the following test should be executed
function check_dolist ()
{
    BASEFILE=`basename ${1}`
    FILECHAR=`echo "${BASEFILE}" | cut -b1`
    if [ "${DONTLIST}" == "${FILECHAR}" ]; then
	echo "YES"
    else
	if [ "${DOLIST}" == "A" ]; then
	    echo "NO"
	else
	    if [ "${DOLIST}" == "${FILECHAR}" ]; then
		echo "NO"
	    else
		echo "YES"
	    fi
	fi
    fi
}

# Clean up
rm -f src/*.o src/*.o1 src/*.pex src/*.err src/*.lst

# Compile and execute all of the selected tests in the src
# directory.  Skip the multiple scripts; they must be handled
# by custom scripts
DONTLIST=5
for file in `ls -1 src/*.pas`; do

    SKIPPING=`check_dolist $file`
    if [ "${SKIPPING}" == "YES" ]; then
	echo "SKIPPING $file"
    else
    echo "########${file}########";
	./testone.sh ${file}
    fi
done

# Special tests performed by custom scripts
DONTLIST=X
for file in ${TSTLST}; do
    SKIPPING=`check_dolist $file`
    if [ "${SKIPPING}" == "YES" ]; then
	echo "SKIPPING $file"
    else
	${file}
    fi
done
exit
