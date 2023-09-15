#!/bin/bash

source /opt/ros/galactic/setup.sh

export SIGNING_TOOL=Tools/cryptotools.py

# Remove old build output
rm -Rf build/ssrc_saluki-v1_default

# Build
make ssrc_saluki-v1_default
