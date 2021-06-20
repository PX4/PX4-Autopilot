#!/usr/bin/env bash
set -e

SCRIPT_DIR=$0
if [[ ${SCRIPT_DIR:0:1} != '/' ]]; then
  SCRIPT_DIR=$(dirname $(realpath -s "$PWD/$0"))
fi

PX4_DIR=$(cd "$(dirname $(dirname $SCRIPT_DIR))" && pwd)

if [ -d $PX4_DIR/build/*_rtps ]; then
  cd $PX4_DIR/build/*_rtps/src/modules/micrortps_bridge/micrortps_agent/
  cmake -Bbuild
  cmake --build build -j$(nproc --all)
fi
