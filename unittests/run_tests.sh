#!/bin/sh

cmake .
make --no-print-directory clean
make --no-print-directory all -j4

set -e

#./param_test
./conversion_test
./autodeclination_test
./mixer_test 2> /dev/null
./sbus2_test
./rc_input_test
