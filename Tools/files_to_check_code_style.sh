#!/usr/bin/env bash
set -eu

PATTERN="-e ."

if [ $# -gt 0 ]
then
    PATTERN="$1"
fi

exec find src \
    -path src/drivers/bootloaders -o \
    -path src/examples/attitude_estimator_ekf -prune -o \
    -path src/examples/ekf_att_pos_estimator -prune -o \
    -path src/lib/DriverFramework -prune -o \
    -path src/lib/ecl -prune -o \
    -path src/lib/external_lgpl -prune -o \
    -path src/lib/mathlib -prune -o \
    -path src/lib/matrix -prune -o \
    -path src/modules/commander -prune -o \
    -path src/modules/sdlog2 -prune -o \
    -path src/modules/systemlib/uthash -prune -o \
    -path src/modules/uavcan -prune -o \
    -path src/modules/uavcan/libuavcan -prune -o \
    -path src/modules/uavcanesc -o \
    -path src/modules/uavcannode -o \
    -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) | grep $PATTERN
