#!/bin/bash
#
# Run container and start test execution
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ -z "$WORKSPACE" ]; then
    echo "\$WORKSPACE not set"
    exit 1
fi

# determine the directory of the source given the directory of this script
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null
ORIG_SRC=$(dirname $SCRIPTPATH)

echo "uploading test logs to Flight Review"
for LOG in `ls $WORKSPACE/test_results/**/*.ulg`
do
    LINK=`$ORIG_SRC/Tools/upload_log.py -q --source CI $LOG`
    echo "Test log: $LINK"
done
