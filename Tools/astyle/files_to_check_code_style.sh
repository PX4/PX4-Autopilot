#!/usr/bin/env bash
set -eu

PATTERN="-e ."

if [ $# -gt 0 ]; then
    PATTERN="$1"
fi

exec find boards msg src platforms \
    -path msg/templates/urtps -prune -o \
    -path platforms/nuttx/NuttX -prune -o \
    -path src/drivers/uavcan/libuavcan -prune -o \
    -path src/lib/DriverFramework -prune -o \
    -path src/lib/ecl -prune -o \
    -path src/lib/matrix -prune -o \
    -path src/lib/systemlib/uthash -prune -o \
    -path src/modules/micrortps_bridge/micro-CDR -prune -o \
    -path src/modules/micrortps_bridge/microRTPS_client -prune -o \
    -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) | grep $PATTERN
