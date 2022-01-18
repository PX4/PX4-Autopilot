#!/bin/bash

source /opt/ros/galactic/setup.sh

export SIGNING_TOOL=Tools/cryptotools.py

# Remove old build output
rm -Rf build/px4_fmu-v5_ssrc build/px4_fmu-v5x_ssrc

# Build
make px4_fmu-v5_ssrc
make px4_fmu-v5x_ssrc
