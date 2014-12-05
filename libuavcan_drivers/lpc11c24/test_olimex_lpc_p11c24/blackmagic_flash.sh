#!/bin/bash
#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
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
