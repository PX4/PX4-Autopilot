#!/bin/bash

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 1. start simulation
# ...

# 2. start maneuver
# ...

# 3. run general tests
LOG_DIR=${SCRIPT_DIR}/../build/posix_sitl_default/logs
LOG_FILE=$( cd ${LOG_DIR} && cd $( ls -t | head -n1 ) && pwd)/$( cd ${LOG_DIR} && cd $( ls -t | head -n1 ) && ls -t | head -n1 )
pytest ../Tools/ulogtests/tests/test_general.py --filepath=${LOG_FILE}