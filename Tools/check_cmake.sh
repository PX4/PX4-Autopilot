#!/bin/bash
cmake_ver=`cmake --version`

if [[ $cmake_ver == "" ]]
then
  exit 1;
fi

if [[ $cmake_ver == *" 2.8"* ]] || [[ $cmake_ver == *" 2.9"* ]] || [[ $cmake_ver == *" 3.0"* ]] || [[ $cmake_ver == *" 3.1"* ]]
then
  exit 1;
fi

exit 0;
