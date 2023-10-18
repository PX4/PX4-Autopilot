#!/bin/bash

source /opt/ros/humble/setup.sh

if [ -z "$1" ]; then
    echo "Usage: $0 <target1 target2 target3..>"
    echo
    exit 1
else
    # go through all given arguments and build them
    for arg in "$@"; do
        echo "BUILDING ${arg}"

        # extract the middle part of the name between the "_"'s
        NAME=${arg}
        NAME=${NAME%_*}
        NAME=${NAME##*_}

        # for our own HW, use HW specific siging. For pixhawks, and icicle board,
        # use the PX4 default signing script and keys
        if [[ $NAME = saluki* ]]
        then
            export SIGNING_TOOL=Tools/saluki-sec-scripts/ed25519_sign.py

            if [ -z "$SIGNING_ARGS" ]; then
                export SIGNING_ARGS=Tools/saluki-sec-scripts/test_keys/$NAME/ed25519_test_key.pem
            fi
        else
            export SIGNING_TOOL=Tools/cryptotools.py
            unset SIGNING_ARGS
        fi

        # Remove old build output
        rm -Rf build/${arg}
        # Build
        make ${arg}

        if [ -n "$SIGNING_ARGS" ]; then
            echo "Signing key: $SIGNING_ARGS"
        fi
    done
fi
