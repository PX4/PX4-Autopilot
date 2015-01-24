#!/bin/bash
d=$PWD && mkdir -p $d/build_arm && cd $d/build_arm && cmake -DNUTTX_BUILD_THREADS="" -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-arm-none-eabi.cmake .. && time make link_exports -j
