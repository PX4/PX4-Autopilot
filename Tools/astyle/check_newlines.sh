#!/usr/bin/env bash

return_value=0

# Check if there are files checked in that don't end in a newline (POSIX requirement)
git grep --cached -Il '' | xargs -L1 bash -c 'if test "$(tail -c 1 "$0")"; then echo "No new line at end of $0"; exit 1; fi'

if [ $? -ne 0 ]; then
	return_value=1
fi

# Check if there are files checked in that have duplicate newlines at the end (fail trailing whitespace checks)
git grep --cached -Il '' | xargs -L1 bash -c 'if tail -c 2 "$0" | ( read x && read y && [ x"$x" = x ] && [ x"$y" = x ]); then echo "Multiple newlines at the end of $0"; exit 1; fi'

if [ $? -ne 0 ]; then
	return_value=1
fi

exit $return_value
