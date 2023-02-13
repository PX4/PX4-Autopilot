#!/bin/bash

echo "*** Starting qurt slpi build ***"

source /home/build-env.sh

make modalai_voxl2-slpi

cat build/modalai_voxl2-slpi_default/src/lib/version/build_git_version.h

echo "*** End of qurt slpi build ***"
