#!/bin/bash

echo "*** Starting qurt build ***"

source /home/build-env.sh

make modalai_voxl2_qurt

cat build/modalai_voxl2_default/src/lib/version/build_git_version.h

echo "*** End of qurt build ***"
