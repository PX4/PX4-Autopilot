#!/bin/bash
#################################################################################
# NxWidgets/Doxygen/gendoc.sh
#
#   Copyright (C) 2012 Gregory Nutt. All rights reserved.
#   Author: Jose Pablo Carballo <jcarballo@nx-engineering.com>
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
    echo "USAGE: $0 <doxygen-output-directory-path>"
    echo ""
    echo "Where:"
    echo " <doxygen-output-directory-path> is the full, absolut path to place the doxygen output"
    echo ""
}

# Input parameters

DOXYGENOUTPUT_DIR=$1
if [ -z "${DOXYGENOUTPUT_DIR}" ]; then
    echo "Missing required arguments"
    ShowUsage
    exit 1
fi

# Check that the directory exist

if [ ! -d "${DOXYGENOUTPUT_DIR}" ]; then
    echo "Directory ${DOXYGENOUTPUT_DIR} does not exist"
    exit 1
fi

# Find the doxygen configuration file

DOXYFILE="Doxyfile"
if [ ! -e "${DOXYFILE}" ]; then
    echo "This script must be executed in the documentation/ directory"
    exit 1
fi

doxygen "${DOXYFILE}" || \
    {
        echo "Failed to run doxygen"; \
        exit 1;
    
    }

cp -rf html "${DOXYGENOUTPUT_DIR}" || \
    {
        echo "Failed to move html output"; \
        exit 1;
    }

rm -rf html || \
    {
        echo "Failed to remove the html/ directory"; \
        exit 1;
    }

echo "open ${DOXYGENOUTPUT_DIR}/html/index.html to start browsing"

