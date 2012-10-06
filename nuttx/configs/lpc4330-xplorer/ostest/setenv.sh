#!/bin/bash
# configs/lpc4330-xplorer/ostest/setenv.sh
#
#   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

if [ "$_" = "$0" ] ; then
  echo "You must source this script, not run it!" 1>&2
  exit 1
fi

WD=`pwd`
if [ ! -x "setenv.sh" ]; then
  echo "This script must be executed from the top-level NuttX build directory"
  exit 1
fi

if [ -z "${PATH_ORIG}" ]; then
  export PATH_ORIG="${PATH}"
fi

# This is the Cygwin path to the location where I installed the Code Red
# toolchain under windows.  You will have to edit this if you install the
# Code Red toolchain in any other location or if you install a different
# version
export TOOLCHAIN_BIN="/cygdrive/c/code_red/RedSuite_4.2.3_379/redsuite/Tools/bin"
#export SCRIPT_BIN="/cygdrive/c/code_red/RedSuite_4.2.3_379/redsuite/bin"
export SCRIPT_BIN="/cygdrive/c/nxp/LPCXpresso_4.2.3_292/lpcxpresso/bin"

# This is the Cygwin path to the location where I installed the CodeSourcery
# toolchain under windows.  You will also have to edit this if you install
# the CodeSourcery toolchain in any other location
#export TOOLCHAIN_BIN="/cygdrive/c/Program Files (x86)/CodeSourcery/Sourcery G++ Lite/bin"
#export SCRIPT_BIN=

# These are the Cygwin paths to the locations where I installed the Atollic
# toolchain under windows.  You will also have to edit this if you install
# the Atollic toolchain in any other location.  /usr/bin is added before
# the Atollic bin path because there is are binaries named gcc.exe and g++.exe
# at those locations as well.
#export TOOLCHAIN_BIN="/usr/bin:/cygdrive/c/Program Files (x86)/Atollic/TrueSTUDIO for ARM Pro 2.3.0/ARMTools/bin"
#export TOOLCHAIN_BIN="/usr/bin:/cygdrive/c/Program Files (x86)/Atollic/TrueSTUDIO for STMicroelectronics STM32 Lite 2.3.0/ARMTools/bin"
#export SCRIPT_BIN=

# This is the Cygwin path to the location where I build the buildroot
# toolchain.
#export TOOLCHAIN_BIN="${WD}/../misc/buildroot/build_arm_nofpu/staging_dir/bin"
#export SCRIPT_BIN=

# And add the selected toolchain path[s] to the PATH variable

export PATH="/sbin:/usr/sbin:${PATH_ORIG}"

if [ ! -z ${SCRIPT_BIN} ]; then
  export PATH="${SCRIPT_BIN}:${PATH}"
fi

export PATH="${TOOLCHAIN_BIN}:${PATH}"
echo "PATH : ${PATH}"

# Set an alias that can be used to put the LPC43xx in boot mode

alias lpc43xx='${SCRIPT_BIN}/Scripts/bootLPCXpresso.cmd winusb'
