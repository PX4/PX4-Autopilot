#!/bin/bash
# This script is meant to be used by the build_all.yml workflow in a github runner
# Please only modify if you know what you are doing
set -e

targets=$1
json_output="["
for target in ${targets//,/ }
do
    json_output="${json_output}'${target}',"
done
json_output="PATH_TO_TARGETS=${json_output}'']"
# echo "${json_output}" >> $GITHUB_ENV
echo $json_output
# build/**/*.bin
# build/**/*.px4
# build/**/*.json
# build/**/events/*.xz
