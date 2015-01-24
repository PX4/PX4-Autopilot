#!/bin/bash
d=$PWD && \
mkdir -p $d/build_arm && cd $d/build_arm && cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-arm-none-eabi.cmake .. && make link_exports -j && ctest  && cpack -G ZIP && \
mkdir -p $d/build_host && cd $d/build_host && cmake .. && make link_exports -j && ctest
