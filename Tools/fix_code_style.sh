#!/bin/bash

if [[ $# -eq 0 ]] ; then
  exit 0
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
astyle \
    --options=$DIR/astylerc          \
    --preserve-date             \
    $*
