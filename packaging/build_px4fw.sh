#!/bin/bash

source /opt/ros/humble/setup.sh

if [ -z "$1" ]; then
    echo "Usage: $0 <target1 target2 target3..>"
    echo
    exit 1
else
    # go through all given arguments and build them
    for arg in "$@"; do
        echo "BUILDING $1"

        # extract the middle part of the name between the "_"'s
        NAME=$1
        NAME=${NAME%_*}
        NAME=${NAME##*_}

        # for our own HW, use HW specific siging. For pixhawks, and icicle board,
        # use the PX4 default signing script and keys
        if [[ $NAME = saluki* ]]
        then
            export SIGNING_TOOL=boards/ssrc/saluki-v1/tools/ed25519_sign.py
            export SIGNING_ARGS=boards/ssrc/$NAME/tools/ed25519_test_key.pem
        else
            export SIGNING_TOOL=Tools/cryptotools.py
            unset SIGNING_ARGS
        fi

        # Remove old build output
        rm -Rf build/$1
        # Build
        make $1
    done
fi
