#!/bin/bash
do_format=$1
files_to_format="""
EKF/AlphaFilter.hpp
EKF/RingBuffer.h
EKF/vel_pos_fusion.cpp
EKF/imu_down_sampler.*pp
mathlib/*.cpp
mathlib/*.h
"""
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

if ! hash clang-format-6.0
then
    echo -e ${RED}Error: No clang-format-6.0 installed${NC}
    exit 1
fi

if [[ $do_format -eq 1 ]]
then
    # formatting
    clang-format-6.0 -i -style=file ${files_to_format}
    echo -e ${GREEN}Formatting finished${NC}
else
    # checking format...
    clang-format-6.0 -style=file  -output-replacements-xml ${files_to_format} | grep -c "<replacement " > /dev/null
    if [[ $? -eq 0 ]]
    then
        echo -e ${RED}Error: need to format${NC}
        echo -e ${YELLOW}From cmake build directory run: 'make format'${NC}
        exit 1
    fi
    echo -e ${GREEN}no formatting needed${NC}
    exit 0
fi
