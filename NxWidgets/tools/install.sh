#!/bin/bash
#################################################################################
# NxWidgets/tools/install.sh
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
#
# set -x
# Functions

function ShowUsage()
{
	echo ""
	echo "Install a unit test in the NuttX source tree"
	echo ""
	echo "USAGE: $0 <apps-directory-path> <test-sub-directory>"
	echo ""
	echo "Where:"
	echo "  <apps-directory-path> is the full, absolute path to the NuttX apps/ directory"
	echo "  <test-sub-directory> is the name of a sub-directory in the UnitTests directory"
	echo ""
}

function ShowTests()
{
	for testdir in ${UNITTEST_DIRPATH}/*; do
		subdir=`basename ${testdir}`
		if [ -d "${UNITTEST_DIRPATH}/${subdir}" ]; then
			if [ -f "${UNITTEST_DIRPATH}/${subdir}/Makefile" ]; then
				echo $subdir
			fi
		fi
	done
}

# Input parameters

APPS_DIRPATH=$1
TEST_SUBDIR=$2

if [ -z "${APPS_DIRPATH}" ]; then
	echo "Missing required arguments"
	ShowUsage
	exit 1
fi

if [ -z "${TEST_SUBDIR}" ]; then
	echo "Missing required argument <test-sub-directory>"
	ShowUsage
	exit 1
fi

# Make sure that we know where we are and where we are going

WD=`pwd`
if [ -x install.sh ]; then
	UNITTEST_DIRPATH="${WD}/../UnitTests"
	TOOLS_DIRPATH="${WD}"
else
	if [ -x tools/install.sh ]; then
		UNITTEST_DIRPATH="${WD}/UnitTests"
		TOOLS_DIRPATH="${WD}/tools"
	else
		echo "This script must be executed in the NxWidgets or NxWidgets/tools directory"
		ShowUsage
		exit 1
	fi
fi

if [ ! -d "${APPS_DIRPATH}" ]; then
	echo "Directory ${APPS_DIRPATH} does not exist"
	ShowUsage
	exit 1
fi

if [ ! -f "${APPS_DIRPATH}/Makefile" ]; then
	echo "Directory ${APPS_DIRPATH} does not look like a NuttX apps directory"
	ShowUsage
	exit 1
fi

TEST_PATH="${UNITTEST_DIRPATH}/${TEST_SUBDIR}"
if [ ! -d "${TEST_PATH}" ]; then
	echo "Directory ${TEST_PATH} does not exist"
	ShowUsage
	ShowTests
	exit 1
fi

if [ ! -f "${TEST_PATH}/Makefile" ]; then
	echo "Directory ${TEST_PATH} does not look like a unit test directory"
	ShowUsage
	ShowTests
	exit 1
fi

# Check if the symbolic link "external" exists in the NuttX apps directory

if [ -e "${APPS_DIRPATH}/external" ]; then
	echo "${APPS_DIRPATH}/external already exists..."
	if [ -h "${APPS_DIRPATH}/external" ]; then
		echo "  Removing the old symbolic link."
		rm "${APPS_DIRPATH}/external" || \
			{ echo "  ERROR: Failed to remove old symbolic link"; \
			  exit 1;
			}
	else
		echo "  ERROR:  But it is not a symbolic link!"
		echo "          Please remove ${APPS_DIRPATH}/external"
		echo "          and run this script again"
	fi
fi

# Then set up the symbolic link "external" in the NuttX apps to point to the
# UnitTest subdirectory

echo "Creating symbolic link"
echo " - To ${TEST_PATH}"
echo " - At ${APPS_DIRPATH}/external"

ln -s "${TEST_PATH}" "${APPS_DIRPATH}/external" || \
	{ echo "Failed to create symbollic link"; \
	  exit 1;
	}

