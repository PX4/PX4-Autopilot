#!/bin/bash

# Script to run ShellCheck (a static analysis tool for shell scripts) over a
# script directory

if [ -z "$1" ]; then
	echo "usage: $0 <directory>"
	echo ""
	echo "  <directory>     Directory to search for scripts"
	exit -1
fi
search_directory="$1"

command -v shellcheck >/dev/null 2>&1 || { echo -e >&2 \
"Error: shellcheck required but it's not installed. On Ubuntu use:\n sudo apt-get install shellcheck\n\nAborting."; exit 1; }

scripts="$(find "$search_directory" -type f ! -name '*.txt' ! -name '*.mix' ! -name '*.bin')"

echo "Running shellcheck in '$search_directory'."

# Disabled rules:
# SC2121: allow 'set' as assignment (NuttX-style)
# SC1008: unrecognized shebang
# SC2086: double quote to prevent globbing and word splitting
# SC2166: allow the form [ $OUTPUT_MODE == fmu -o $OUTPUT_MODE == io ]
# SC2148: allow files w/o shebang
# SC2039: In POSIX sh, array references are undefined. TODO: fix this
shellcheck -x \
	-e SC1008 \
	-e SC2086 \
	-e SC2121 \
	-e SC2148 \
	-e SC2166 \
	-e SC2039 \
	$scripts
ret=$?
if [ $ret -ne 0 ]; then
	echo "Please fix the above script problems."
	echo "If an error is raised that should be ignored, \
add the following right before the offending line:"
	echo "# shellcheck disable=SCxxxx"
	echo ""
	echo "Re-run the script with '$0 $@'"
	exit $ret
fi

echo "No problems found."
exit 0
