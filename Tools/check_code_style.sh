#!/usr/bin/env bash
set -eu
failed=0
for fn in $(find . -path './src/lib/uavcan' -prune -o \
                   -path './src/lib/mathlib/CMSIS' -prune -o \
                   -path './src/lib/eigen' -prune -o \
                   -path './src/modules/attitude_estimator_ekf/codegen' -prune -o \
                   -path './NuttX' -prune -o \
                   -path './Build' -prune -o \
                   -path './mavlink' -prune -o \
                   -path './unittests/gtest' -prune -o \
                   -path './unittests/build' -prune -o \
                   -name '*.c' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -type f); do
  if [ -f "$fn" ];
  then
    ./Tools/fix_code_style.sh --quiet < $fn > $fn.pretty
    diffsize=$(diff -y --suppress-common-lines $fn $fn.pretty | wc -l)
    rm -f $fn.pretty
    if [ $diffsize -ne 0 ]; then
      failed=1
      echo $diffsize $fn
    fi
  fi
done

if [ $failed -eq 0 ]; then
    echo "Format checks passed"
    exit 0
else
    echo "Format checks failed; please run ./Tools/fix_code_style.sh on each file"
    exit 1
fi
