#!/bin/sh

make clean
make all
./mixer_test
./sbus2_test ../../../../data/sbus2/sbus2_r7008SB_gps_baro_tx_off.txt