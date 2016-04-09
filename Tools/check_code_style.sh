#!/usr/bin/env bash
set -eu
failed=0
for fn in $(find src/examples \
                 src/systemcmds \
                 src/include \
                 src/drivers \
                 src/platforms \
                 src/firmware \
                 src/lib/launchdetection \
                 src/lib/geo \
                 src/lib/geo_lookup \
                 src/lib/conversion \
                 src/lib/rc \
                 src/lib/version \
                 src/modules/attitude_estimator_q \
                 src/modules/fw_att_control \
                 src/modules/gpio_led \
                 src/modules/land_detector \
                 src/modules/muorb \
                 src/modules/px4iofirmware \
                 src/modules/param \
                 src/modules/sensors \
                 src/modules/simulator \
                 src/modules/uORB \
                 src/modules/bottle_drop \
                 src/modules/dataman \
                 src/modules/segway \
                 src/modules/local_position_estimator \
                 src/modules/unit_test \
                 src/modules/systemlib \
                 src/modules/controllib \
                   -path './Build' -prune -o \
                   -path './mavlink' -prune -o \
                   -path './NuttX' -prune -o \
                   -path './src/lib/eigen' -prune -o \
                   -path './src/modules/uavcan/libuavcan' -prune -o \
                   -path './src/modules/attitude_estimator_ekf/codegen' -prune -o \
                   -path './src/modules/ekf_att_pos_estimator' -prune -o \
                   -path './src/modules/sdlog2' -prune -o \
                   -path './src/modules/uORB' -prune -o \
                   -path './src/modules/vtol_att_control' -prune -o \
                   -path './unittests/build' -prune -o \
                   -path './unittests/gtest' -prune -o \
                   -name '*.c' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \
                   -not -name '*generated*' \
                   -not -name '*uthash.h' \
                   -not -name '*utstring.h' \
                   -not -name '*utlist.h' \
                   -not -name '*utarray.h' \
                   -type f); do
  if [ -f "$fn" ];
  then
    ./Tools/fix_code_style.sh --quiet < $fn > $fn.pretty
    diffsize=$(diff -y --suppress-common-lines $fn $fn.pretty | wc -l)
    rm -f $fn.pretty
    if [ $diffsize -ne 0 ]; then
      failed=1
      echo $fn 'bad formatting, please run "./Tools/fix_code_style.sh' $fn'"'
    fi
  fi
done

if [ $failed -eq 0 ]; then
    echo "Format checks passed"
    exit 0
else
    echo "Format checks failed"
    exit 1
fi
