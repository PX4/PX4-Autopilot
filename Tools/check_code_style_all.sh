#!/usr/bin/env bash
set -eu
failed=0

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

find \
    src/drivers \
    src/examples \
    src/firmware \
    src/include \
    src/lib/controllib \
    src/lib/conversion \
    src/lib/geo \
    src/lib/geo_lookup \
    src/lib/launchdetection \
    src/lib/rc \
    src/lib/tailsitter_recovery \
    src/lib/terrain_estimation \
    src/lib/version \
    src/modules/attitude_estimator_q \
    src/modules/bottle_drop \
    src/modules/controllib_test \
    src/modules/dataman \
    src/modules/fw_att_control \
    src/modules/fw_pos_control_l1 \
    src/modules/gpio_led \
    src/modules/land_detector \
    src/modules/local_position_estimator \
    src/modules/logger \
    src/modules/mavlink/mavlink_tests \
    src/modules/muorb \
    src/modules/param \
    src/modules/px4iofirmware \
    src/modules/replay \
    src/modules/segway \
    src/modules/sensors \
    src/modules/simulator \
    src/modules/systemlib \
    src/modules/unit_test \
    src/modules/uORB \
    src/modules/vtol_att_control \
    src/platforms \
    src/systemcmds \
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
