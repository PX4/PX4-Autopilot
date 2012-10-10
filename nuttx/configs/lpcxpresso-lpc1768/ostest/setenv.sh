#!/bin/bash
# configs/lpcxpresso-lpc1768/ostest/setenv.sh
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

if [ "$(basename $0)" = "setenv.sh" ] ; then
  echo "You must source this script, not run it!" 1>&2
  exit 1
fi

if [ -z "${PATH_ORIG}" ]; then export PATH_ORIG="${PATH}"; fi

WD=`pwd`

# This is where the buildroot might reside on a Linux or Cygwin system
# export TOOLCHAIN_BIN="${WD}/../misc/buildroot/build_arm_nofpu/staging_dir/bin"

# This is the default install location for Code Red on Linux
export TOOLCHAIN_BIN="/usr/local/LPCXpresso/tools/bin"

# This is the Cygwin path to the LPCXpresso 3.6 install location under Windows
#export TOOLCHAIN_BIN="/cygdrive/c/nxp/lpcxpresso_3.6/Tools/bin"

# This is the path to the LPCXpression tool subdirectory
export LPCTOOL_DIR="${WD}/configs/lpcxpresso-lpc1768/tools"

# Add the path to the toolchain to the PATH varialble
export PATH="${TOOLCHAIN_BIN}:${LPCTOOL_DIR}:/sbin:/usr/sbin:${PATH_ORIG}"

echo "PATH : ${PATH}"
