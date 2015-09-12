#!/bin/bash

mkdir -p build_posix_sitl_simple/rootfs/fs/microsd
mkdir -p build_posix_sitl_simple/rootfs/eeprom
cd build_posix_sitl_simple/src/firmware/posix && ./mainapp ../../../../$1
