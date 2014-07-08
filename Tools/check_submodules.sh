#!/bin/sh

[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && {
    # GIT_SUBMODULES_ARE_EVIL is set, meaning user doesn't want submodules
    echo "Skipping submodules. NUTTX_SRC is set to $NUTTX_SRC"
    exit 0
}

if [ -d NuttX/nuttx ];
	then
	STATUSRETVAL=$(git status --porcelain | grep -i "NuttX")
	if [ "$STATUSRETVAL" == "" ]; then
		echo "Checked NuttX submodule, correct version found"
	else
		echo "NuttX sub repo not at correct version. Try 'make updatesubmodules'"
		echo "or follow instructions on http://pixhawk.org/dev/git/submodules"
		exit 1
	fi
else
	git submodule init;
	git submodule update;
fi


if [ -d mavlink/include/mavlink/v1.0 ];
	then
	STATUSRETVAL=$(git status --porcelain | grep -i "mavlink/include/mavlink/v1.0")
	if [ "$STATUSRETVAL" == "" ]; then
		echo "Checked mavlink submodule, correct version found"
	else
		echo "mavlink sub repo not at correct version. Try 'make updatesubmodules'"
		echo "or follow instructions on http://pixhawk.org/dev/git/submodules"
		exit 1
	fi
else
	git submodule init;
	git submodule update;
fi

exit 0
