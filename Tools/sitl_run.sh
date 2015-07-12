#!/bin/bash

mkdir -p Build/posix_sitl.build/rootfs/fs/microsd
mkdir -p Build/posix_sitl.build/rootfs/eeprom
cd Build/posix_sitl.build && ./mainapp ../../$1
