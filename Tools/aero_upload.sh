#!/bin/bash
############################################################################
#
#   Copyright (C) 2016  Intel Corporation. All rights reserved.
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
# 3. Neither the name PX4 nor the names of its contributors may be
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

set -e

USER=${AERO_USER:-root}
HOSTNAME=${AERO_HOSTNAME:-intel-aero.local}
SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

target=$USER@$HOSTNAME
firmware=$1
px_uploader=${SCRIPT_DIR}/px_uploader.py

echo "Copying files to Aero board ($target)..."
scp $firmware $px_uploader $target:

ssh $target /bin/bash <<EOF
    uname -a
    /usr/sbin/get_aero_version.py
    router_running=0
    if [ -n "\$(fuser /dev/ttyS1)" ]; then
        router_running=1
        # try stopping router
        /etc/init.d/mavlink-routerd.sh stop
        p=\$(fuser /dev/ttyS1)
        if [ -n "\$p" ]; then
            echo "Process \$p is running and keeping UART busy"
            exit 1
        fi
    fi
    echo -e "Updating firmware on AeroFC"
    ~/px_uploader.py \
        --port /dev/ttyS1 \
        --baud-flightstack 1500000,460800,115200 \
        $(basename $firmware)
    echo "Firmware updated"
    if [ \$router_running -eq 1 ]; then
        echo "Restarting router"
        /etc/init.d/mavlink-routerd.sh start
    fi
EOF

echo "Finished."
