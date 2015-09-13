#!/bin/bash

cd build_posix_sitl_simple/src/firmware/posix
mkdir -p rootfs/fs/microsd
mkdir -p rootfs/eeprom
touch rootfs/eeprom/parameters
./mainapp ../../../../$1
