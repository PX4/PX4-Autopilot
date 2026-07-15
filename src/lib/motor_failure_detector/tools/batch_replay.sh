#!/usr/bin/env bash
# Run mfd_replay on every .ulg in a folder and summarize each verdict.
#
# Usage:  batch_replay.sh <log-dir> [--set MOTFAIL_NAME=value ...]
#   env MFD_REPLAY=/path/to/mfd_replay   overrides the binary location
#
# For a very large fleet, parallelize instead:  ls DIR/*.ulg | xargs -P"$(nproc)" -n1 mfd_replay
set -uo pipefail

# default: the standard SITL build, relative to the repo root (run this from there)
BIN="${MFD_REPLAY:-build/px4_sitl_default/mfd_replay}"
dir="${1:?usage: batch_replay.sh <log-dir> [--set NAME=value ...]}"
shift   # anything after the dir ("$@") is passed straight through to mfd_replay

shopt -s nullglob   # an empty folder yields no iterations, not a literal "*.ulg"
ok=0 fail=0 skip=0

for log in "$dir"/*.ulg; do
	name=$(basename "$log")
	output=$("$BIN" "$log" "$@" 2>/dev/null)

	case $? in   # mfd_replay exit code: 0 = clean, 1 = a motor tripped, 2 = unreadable / no ESC data
	0) echo "OK    $name"                             ; ok=$((ok + 1)) ;;
	1) motors=$(echo "$output" | awk '/FAILED/ {printf "m%s ", $1}')
	   echo "FAIL  $name  [${motors% }]"              ; fail=$((fail + 1)) ;;
	*) echo "SKIP  $name  (no ESC data / unreadable)" ; skip=$((skip + 1)) ;;
	esac
done

echo "----"
echo "total=$((ok + fail + skip))  OK=$ok  FAIL=$fail  SKIP=$skip"
