#!/usr/bin/env bash
set -eu

if [[ "$@" == "--fix" ]]
then
    export PX4_ASTYLE_FIX=1
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

find src \
    -path src/lib/DriverFramework -prune -o \
    -path src/lib/ecl -prune -o \
    -path src/lib/external_lgpl -prune -o \
    -path src/lib/mathlib -prune -o \
    -path src/lib/matrix -prune -o \
    -path src/modules/attitude_estimator_ekf -prune -o \
    -path src/modules/commander -prune -o \
    -path src/examples/ekf_att_pos_estimator -prune -o \
    -path src/modules/mavlink -prune -o \
    -path src/examples/attitude_estimator_ekf -prune -o \
    -path src/modules/navigator -prune -o \
    -path src/modules/sdlog2 -prune -o \
    -path src/modules/uavcan -prune -o \
    -path src/modules/uavcan/libuavcan -prune -o \
    -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
    -not -name '*generated.h' \
    -not -name '*uthash.h' \
    -not -name '*utstring.h' \
    -not -name '*utlist.h' \
    -not -name '*utarray.h' \
    -print0 | xargs -0 -n 1 -P 8 -I % ${DIR}/check_code_style.sh %


if [ $? -eq 0 ]; then
    echo "Format checks passed"
    exit 0
fi
