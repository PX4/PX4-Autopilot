#!/bin/bash

cd src/modules/muorb/apps/libfc-sensor-api
rm -fR build
mkdir build
cd build
CC=/home/4.1.0.4/tools/linaro64/bin/aarch64-linux-gnu-gcc cmake ..
make
cd ../../../../../..

