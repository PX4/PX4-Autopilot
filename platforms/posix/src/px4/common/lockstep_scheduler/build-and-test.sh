#!/usr/bin/env bash

set -e

rm -rf build

(mkdir -p build && cd build && CXX=g++ cmake .. && make -j8 && time $@ ./test/lockstep_scheduler_test)
rm -rf build

(mkdir -p build && cd build && CXX=clang++ cmake .. && make -j8 && time $@ ./test/lockstep_scheduler_test)
rm -rf build
