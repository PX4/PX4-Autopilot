#!/usr/bin/env bash

function check_git_submodule {

# The .git exists in a submodule if init and update have been done.
if [[ -f $1"/.git" || -d $1"/.git" ]]; then

	# CI environment always update
	if [ "$CI" == "true" ]; then
		git_submodule_update $1
		exit 0
	fi

	SUBMODULE_STATUS=$(git submodule summary "$1")
	STATUSRETVAL=$(echo $SUBMODULE_STATUS | grep -A20 -i "$1")

	echo "SUBMODULE_STATUS: ${SUBMODULE_STATUS}"
	echo "STATUSRETVAL: ${STATUSRETVAL}"

	if ! [[ -z "$STATUSRETVAL" ]]; then



		# check submodule status as of previous commit (HEAD@{1})
		GIT_SHA_PREV=$(git ls-tree -d HEAD@{1} $1 | awk '{print $3;}')
		GIT_SHA_CURR=$(git -C $1 rev-parse HEAD)

		echo "$1 GIT_SHA_PREV: ${GIT_SHA_PREV}"
		echo "$1 GIT_SHA_CURR: ${GIT_SHA_CURR}"

		if [ "${GIT_SHA_PREV}" == "${GIT_SHA_CURR}" ]; then
			# submodule is currently on the wrong commit, but on previous HEAD it wasn't, therefore it's safe to update
			echo "$1 GIT SHA matched previous, updating"
			git_submodule_update $1

		else

			echo -e "\033[31mChecked $1 submodule, ACTION REQUIRED:\033[0m"
			echo ""
			echo -e "Different commits:"
			echo -e "$SUBMODULE_STATUS"
			echo ""
			echo ""
			echo -e " *******************************************************************************"
			echo -e " *   \033[31mIF YOU DID NOT CHANGE THIS FILE (OR YOU DON'T KNOW WHAT A SUBMODULE IS):\033[0m  *"
			echo -e " *   \033[31mHit 'u' and <ENTER> to update ALL submodules and resolve this.\033[0m            *"
			echo -e " *   (performs \033[94mgit submodule sync --recursive\033[0m                                  *"
			echo -e " *    and \033[94mgit submodule update --init --recursive\033[0m )                            *"
			echo -e " *******************************************************************************"
			echo ""
			echo ""
			echo -e "   Only for EXPERTS:"
			echo -e "   $1 submodule is not in the recommended version."
			echo -e "   Hit 'y' and <ENTER> to continue the build with this version. Hit <ENTER> to resolve manually."
			echo -e "   Use \033[94mgit add $1 && git commit -m 'Updated $1'\033[0m to choose this version (careful!)"
			echo ""
			read user_cmd
			if [ "$user_cmd" == "y" ]; then
				echo "Continuing build with manually overridden submodule.."
			elif [ "$user_cmd" == "u" ]; then
				git_submodule_update $1

				echo "Submodule fixed, continuing build.."
			else
				echo "Build aborted."
				exit 1
			fi
		fi
	fi
else
	git_submodule_update $1

fi

}

function git_submodule_update {
	git submodule --quiet sync --recursive --quiet -- $1
	git submodule --quiet update --init --recursive --jobs=8 -- $1  || true
	git submodule --quiet update --init --recursive -- $1

	# put submodule on branch
	git -C $1 checkout -q -B $(git config -f .gitmodules submodule.$1.branch || echo master)
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
