#!/bin/bash

mkdir -p Build/posix_default.build/rootfs/fs/microsd
mkdir -p Build/posix_default.build/rootfs/eeprom
cd Build/posix_default.build && ./mainapp ../../posix-configs/posixtest/init/rc.S
