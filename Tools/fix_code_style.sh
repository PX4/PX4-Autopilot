#!/bin/bash

ASTYLE_VER=`astyle --version`
ASTYLE_VER_REQUIRED="Artistic Style Version 2.05.1"

if [ "$ASTYLE_VER" != "$ASTYLE_VER_REQUIRED" ]; then
  echo "Error: you're using ${ASTYLE_VER}, but PX4 requires ${ASTYLE_VER_REQUIRED}"
  echo "You can get the correct version here: https://github.com/PX4/astyle/releases/tag/2.05.1"
  exit 1
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
astyle \
    --options=$DIR/astylerc          \
    --preserve-date             \
    $*
