#!/bin/sh
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
astyle \
    --options=$DIR/astylerc          \
    --preserve-date             \
    $*
