#!/bin/bash

echo "*** Starting apps processor build ***"

source /home/build-env.sh

make modalai_voxl2

cat build/modalai_voxl2_default/src/lib/version/build_git_version.h

echo "*** End of apps processor build ***"
