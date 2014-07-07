#!/bin/sh
STATUSRETVAL=$(git status --porcelain | grep -i "M mavlink/include/mavlink/v1.0")
if [ "$STATUSRETVAL" == "" ]; then
	echo "checked mavlink submodule, correct version found"
else
	echo "mavlink sub repo not at correct version. Try 'git submodule update'"
	exit 1
fi

exit 0
