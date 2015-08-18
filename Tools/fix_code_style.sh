#!/bin/sh
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
astyle \
    --options=$DIR/Tools/astylerc          \
    --preserve-date             \
    $*
