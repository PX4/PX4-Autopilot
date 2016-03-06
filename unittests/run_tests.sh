#!/bin/sh

cmake .
make clean
make all -j4

set -e

#./param_test
./conversion_test
./autodeclination_test
./mixer_test > /dev/null
./sbus2_test
./rc_input_test
