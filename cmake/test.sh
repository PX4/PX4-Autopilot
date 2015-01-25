#!/bin/bash
d=$PWD && \
mkdir -p $d/build_arm && cd $d/build_arm && cmake .. && make -j && ctest  && cpack -G ZIP && \
mkdir -p $d/build_host && cd $d/build_host && cmake .. -DHOST_TEST=ON && make -j && ctest
