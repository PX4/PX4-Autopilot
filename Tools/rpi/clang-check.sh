#!/bin/bash
# This script is specific for Raspberry Pi
COMPILE_DB=$(/bin/pwd)/build_posix_rpi_cross;
BIN=${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin

CXX_INC=$(cd ${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/include/c++/*; pwd)

EXTRA_ARG1=-I${CXX_INC}
EXTRA_ARG2=-I${CXX_INC}/arm-linux-gnueabihf
EXTRA_ARG3=-I${CXX_INC}/backward

grep file ${COMPILE_DB}/compile_commands.json |
awk '{ print $2; }' |
sed 's/\"//g' |
while read FILE; do
    (cd $(dirname ${FILE});
    ${BIN}/clang-check -analyze -p ${COMPILE_DB} --extra-arg=${EXTRA_ARG1} --extra-arg=${EXTRA_ARG2} --extra-arg=${EXTRA_ARG3} $(basename ${FILE})
    );
  done
