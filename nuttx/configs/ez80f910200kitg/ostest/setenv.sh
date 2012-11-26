#!/bin/bash
# configs/ez80f910200kitg/ostest/setenv.sh
#
#   Copyright (C) 2008, 2009 Gregory Nutt. All rights reserved.
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
# Check how we were executed
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

#
# This is the Cygwin path to location where the XDS-II tools were installed
#
TOOLCHAIN_BIN="/cygdrive/c/Program Files (x86)/ZiLOG/ZDSII_eZ80Acclaim!_5.1.1\bin"

#
# Add the path to the toolchain to the PATH varialble.  NOTE that /bin and /usr/bin
# preceded the toolchain bin directory.  This is because the ZDSII bin directory
# includes binaries like make.exe that will interfere with the normal build process
# if we do not give priority to the versions at /bin and /usr/bin.
#
export PATH="/bin:/usr/bin:${TOOLCHAIN_BIN}:/sbin:/usr/sbin:${PATH_ORIG}"
echo "PATH : ${PATH}"
