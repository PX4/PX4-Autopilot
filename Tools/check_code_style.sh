#!/usr/bin/env bash
set -eu
failed=0
for fn in $(find src/examples \
                 src/systemcmds \
                 src/include \
                 src/drivers/blinkm \
                 src/drivers/bma180 \
                 src/drivers/pca9685 \
                 src/drivers/pca8574 \
                 src/drivers/md25 \
                 src/drivers/ms5611 \
                 src/drivers/stm32 \
                 src/drivers/px4io \
                 src/drivers/px4fmu \
                 src/lib/launchdetection \
                 src/modules/bottle_drop \
                 src/modules/dataman \
                 src/modules/fixedwing_backside \
                 src/modules/segway \
                 src/modules/unit_test \
                 src/modules/systemlib \
                   -path './Build' -prune -o \
                   -path './mavlink' -prune -o \
                   -path './NuttX' -prune -o \
                   -path './src/lib/eigen' -prune -o \
                   -path './src/lib/mathlib/CMSIS' -prune -o \
                   -path './src/lib/uavcan' -prune -o \
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
