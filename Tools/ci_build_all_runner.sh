#!/bin/bash
# This script is meant to be used by the build_all.yml workflow in a github runner
# Please only modify if you know what you are doing

targets=$1
for target in ${targets//,/ }
do
    make $target
done
