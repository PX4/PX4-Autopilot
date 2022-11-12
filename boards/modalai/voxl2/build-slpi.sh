#!/bin/bash

echo "*** Starting qurt build ***"

source /home/build-env.sh

make modalai_voxl2_slpi

cat build/modalai_voxl2_slpi_default/src/lib/version/build_git_version.h

echo "*** End of qurt build ***"
