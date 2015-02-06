#!/bin/sh

[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && {
    # GIT_SUBMODULES_ARE_EVIL is set, meaning user doesn't want submodules
    echo "Skipping submodules. NUTTX_SRC is set to $NUTTX_SRC"
    exit 0
}

if [ -d NuttX/nuttx ];
	then
	STATUSRETVAL=$(git submodule summary | grep -A20 -i "NuttX" | grep "<")
	if [ -z "$STATUSRETVAL" ]; then
		echo "Checked NuttX submodule, correct version found"
	else
		echo ""
		echo ""
		echo "New commits required:"
		echo "$(git submodule summary)"
		echo ""
		echo ""
		echo "   NuttX sub repo not at correct version. Try 'git submodule update'"
		echo "   or follow instructions on http://pixhawk.org/dev/git/submodules"
		echo ""
		echo "   DO NOT FORGET TO RUN 'make distclean && make archives' AFTER EACH NUTTX UPDATE!"
		exit 1
	fi
else
	git submodule init;
	git submodule update;
fi


if [ -d mavlink/include/mavlink/v1.0 ];
	then
	STATUSRETVAL=$(git submodule summary | grep -A20 -i "mavlink/include/mavlink/v1.0" | grep "<")
	if [ -z "$STATUSRETVAL" ]; then
		echo "Checked mavlink submodule, correct version found"
	else
		echo ""
		echo ""
		echo "New commits required:"
		echo "$(git submodule summary)"
		echo ""
		echo ""
		echo "mavlink sub repo not at correct version. Try 'git submodule update'"
		echo "or follow instructions on http://pixhawk.org/dev/git/submodules"
		exit 1
	fi
else
	git submodule init;
	git submodule update;
fi


if [ -d uavcan ]
then
	STATUSRETVAL=$(git submodule summary | grep -A20 -i uavcan | grep "<")
	if [ -z "$STATUSRETVAL" ]
	then
		echo "Checked uavcan submodule, correct version found"
	else
		echo ""
		echo ""
		echo "New commits required:"
		echo "$(git submodule summary)"
		echo ""
		echo ""
		echo "uavcan sub repo not at correct version. Try 'git submodule update'"
		echo "or follow instructions on http://pixhawk.org/dev/git/submodules"
		exit 1
	fi
else
	git submodule init;
	git submodule update;
fi

if [ -d Tools/gencpp ]
then
	STATUSRETVAL=$(git submodule summary | grep -A20 -i gencpp | grep "<")
	if [ -z "$STATUSRETVAL" ]
	then
		echo "Checked gencpp submodule, correct version found"
	else
		echo ""
		echo ""
		echo "New commits required:"
		echo "$(git submodule summary)"
		echo ""
		echo ""
		echo "gencpp sub repo not at correct version. Try 'git submodule update'"
		echo "or follow instructions on http://pixhawk.org/dev/git/submodules"
		exit 1
	fi
else
	git submodule init;
	git submodule update;
fi

if [ -d Tools/genmsg ]
then
	STATUSRETVAL=$(git submodule summary | grep -A20 -i genmsg | grep "<")
	if [ -z "$STATUSRETVAL" ]
	then
		echo "Checked genmsg submodule, correct version found"
	else
		echo ""
		echo ""
		echo "New commits required:"
		echo "$(git submodule summary)"
		echo ""
		echo ""
		echo "genmsg sub repo not at correct version. Try 'git submodule update'"
		echo "or follow instructions on http://pixhawk.org/dev/git/submodules"
		exit 1
	fi
else
	git submodule init;
	git submodule update;
fi

exit 0
