#!/bin/bash

echo "*** Starting posix build ***"

source /home/build-env.sh

make modalai_voxl2_default

cat build/modalai_voxl2_default/src/lib/version/build_git_version.h


echo "*** End of posix build ***"
