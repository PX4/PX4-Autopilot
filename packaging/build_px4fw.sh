#!/bin/bash

source /opt/ros/galactic/setup.sh
export SIGNING_TOOL=Tools/cryptotools.py

if [ -z "$1" ]; then
    echo "Usage: $0 <target1 target2 target3..>"
    echo
    exit 1
else
    # go through all given arguments and build them
    for arg in "$@"; do
        echo "BUILDING $1"
        # Remove old build output
        rm -Rf build/$1
        # Build
        make $1
    done
fi
