#!/usr/bin/env bash

function check_git_submodule {

# The .git exists in a submodule if init and update have been done.
if [[ -f $1"/.git" || -d $1"/.git" ]]; then

	# always update within CI environment or configuring withing VSCode CMake where you can't interact
	if [ "$CI" == "true" ] || [ -n "${VSCODE_PID+set}" ] || [ -n "${CLION_IDE+set}" ]; then
		git submodule --quiet sync --recursive -- $1
		git submodule --quiet update --init --recursive --jobs=8 -- $1  || true
		git submodule --quiet sync --recursive -- $1
		git submodule --quiet update --init --recursive --jobs=8 -- $1
		exit 0
	fi

	SUBMODULE_STATUS=$(git submodule summary "$1")
	STATUSRETVAL=$(echo $SUBMODULE_STATUS | grep -A20 -i "$1")
	if ! [[ -z "$STATUSRETVAL" ]]; then
		echo -e "\033[33mWarning: $1 submodule has uncommitted changes:\033[0m"
		echo -e "$SUBMODULE_STATUS"
		echo ""
		echo -e "To update submodules to the expected version, run:"
		echo -e "   \033[94mgit submodule sync --recursive && git submodule update --init --recursive\033[0m"
		echo ""
	fi
else
	git submodule --quiet sync --recursive --quiet -- $1
	git submodule --quiet update --init --recursive -- $1  || true
	git submodule --quiet sync --recursive --quiet -- $1
	git submodule --quiet update --init --recursive -- $1
fi

}

# If called with a path then respect $GIT_SUBMODULES_ARE_EVIL but do normal processing
if [ "$#" != "0" ]; then
	# called with a path then process only that path but respect $GIT_SUBMODULES_ARE_EVIL
	[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && {
		# GIT_SUBMODULES_ARE_EVIL is set, meaning user doesn't want submodules updated
		exit 0
	}

	check_git_submodule $1

else

	[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && {
		# GIT_SUBMODULES_ARE_EVIL is set, meaning user doesn't want submodules updated
		echo "GIT_SUBMODULES_ARE_EVIL is defined - Skipping All submodule checking!"
		exit 0
	}

	submodules=$(git submodule status | awk '{ print $2 }')
	for i in $submodules;
	do
		check_git_submodule $i
	done

fi

exit 0
