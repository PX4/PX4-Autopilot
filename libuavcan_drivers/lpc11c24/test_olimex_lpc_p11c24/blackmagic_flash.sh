#!/bin/bash
#
# Copyright (c) 2014 Courierdrone, courierdrone.com
# Distributed under the MIT License, available in the file LICENSE.
# Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
#

PORT=${1:-'/dev/ttyACM0'}
#/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_DDE578CC-if00

elf=build/firmware.elf

arm-none-eabi-size $elf || exit 1

tmpfile=fwupload.tempfile
cat > $tmpfile <<EOF
target extended-remote $PORT
mon swdp_scan
attach 1
load
kill
EOF

arm-none-eabi-gdb $elf --batch -x $tmpfile
rm $tmpfile
