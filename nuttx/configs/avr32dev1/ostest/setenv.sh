#!/bin/bash
# configs/avr32dev1/ostest/setenv.sh
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

if [ "$(basename $0)" = "setenv.sh" ] ; then
  echo "You must source this script, not run it!" 1>&2
  exit 1
fi

#
# This PATH setup assumes that you are using versin 2.1.4 of the Atmel
# AVR GNU tools installed at the default location on Windows.  NOTE
# that the path is in appended to the end of the PATH variable; this is
# because there are also many GNUWin32 binaries there that conflict with
# Cygwin versions.
#

if [ -z "${PATH_ORIG}" ]; then export PATH_ORIG="${PATH}"; fi

WD=`pwd`

export AVR32_BIN="/cygdrive/c/Program Files/Atmel/AVR Tools/AVR32 Toolchain/bin/"
export FLIP_BIN="/cygdrive/c/Program Files/Atmel/Flip 3.4.2/bin"
export AVR32DEV1_BIN="${WD}/configs/avr32dev1/tools"
export PATH="${FLIP_BIN}:${AVR32DEV1_BIN}:/sbin:/usr/sbin:${PATH_ORIG}:${AVR32_BIN}"

echo "PATH : ${PATH}"
