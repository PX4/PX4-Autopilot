#!/bin/bash

git submodule init boards/modalai/voxl2/libfc-sensor-api
git submodule update boards/modalai/voxl2/libfc-sensor-api

cd boards/modalai/voxl2/libfc-sensor-api
rm -fR build
mkdir build
cd build
CC=/home/4.1.0.4/tools/linaro64/bin/aarch64-linux-gnu-gcc cmake ..
make
cd ../../../../..

