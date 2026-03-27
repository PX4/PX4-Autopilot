#!/bin/bash

echo "*** Starting unified VOXL2 build (apps + SLPI) ***"

source /home/build-env.sh

make modalai_voxl2_deb

cat build/modalai_voxl2_default/src/lib/version/build_git_version.h

echo "*** End of unified VOXL2 build ***"
