#!/bin/sh
set -e

PX4_BIN=/opt/px4/bin/px4

# if no args passed, default to SITL none
if [ $# -eq 0 ]; then
  exec "$PX4_BIN"
else
  exec "$PX4_BIN" "$@"
fi
