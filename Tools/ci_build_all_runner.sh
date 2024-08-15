#!/bin/bash
# This script is meant to be used by the build_all.yml workflow in a github runner
# Please only modify if you know what you are doing
set -e

echo "### Build Summary" >> $GITHUB_STEP_SUMMARY
targets=$1
for target in ${targets//,/ }
do
    echo "::group::Building: [${target}]"
    start=$(date +%s)
    make $target
    stop=$(date +%s)
    diff=$(($stop-$start))
    build_time="$(($diff /60/60))h $(($diff /60))m $(($diff % 60))s elapsed"
    echo "::notice ::[Build Time]: $build_time."
    echo "* $target - $build_time" >> $GITHUB_STEP_SUMMARY
    echo "::endgroup::"
done
