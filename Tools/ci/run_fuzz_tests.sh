#!/bin/bash
# This script runs the fuzz tests from a given binary for a certain amount of time
set -e

if [[ "$1" == "--help" || "$1" == "-h" || -z "$1" ]]; then
  echo "Usage: $0 <binary> [<duration>]"
  echo "duration can be for example 5m or 5h"
  exit 0
fi

binary="$1"
duration="$2"
[[ -z "$duration" ]] && duration="1m"

# Iterate over all available fuzz tests in the binary
for t in $("$binary" --fuzz=__non_existent__ 2>&1 | sed '1,/^Valid tests:$/d' | tr -d ' '); do
  echo "Running $t"
  "$binary" --fuzz="$t" --fuzz_for="$duration"
done
