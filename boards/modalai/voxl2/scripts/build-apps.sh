#!/bin/bash

echo "*** Starting apps processor build ***"

source /home/build-env.sh

export PATH=/home/4.1.0.4/tools/linaro64/bin:$PATH

make modalai_voxl2

cat build/modalai_voxl2_default/src/lib/version/build_git_version.h

echo "*** End of apps processor build ***"
