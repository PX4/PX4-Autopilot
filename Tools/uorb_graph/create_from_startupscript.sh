#! /bin/bash
# create the graph from a posix (e.g. SITL) startup script

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

startup_file="$SCRIPT_DIR"/../../posix-configs/SITL/init/ekf2/typhoon_h480
[ -n "$1" ] && startup_file=$1
# get the modules as comma-separated list
modules=$(cat "$startup_file"|cut -f1 -d' '|sort|uniq|tr '\n' ,)

cd "$SCRIPT_DIR/../.."
"$SCRIPT_DIR"/create.py --src-path src -m "$modules" -f "$SCRIPT_DIR/graph_runtime_sitl"

