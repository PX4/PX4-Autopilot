#!/usr/bin/env bash

[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && {
    # GIT_SUBMODULES_ARE_EVIL is set, meaning user doesn't want submodules
    echo "Skipping submodules. NUTTX_SRC is set to $NUTTX_SRC"
    exit 0
}


GITSTATUS=$(git status)

function check_git_submodule {

# The .git exists in a submodule if init and update have been done.
if [ -f $1"/.git" ];
	then
	SUBMODULE_STATUS=$(git submodule summary "$1")
	STATUSRETVAL=$(echo $SUBMODULE_STATUS | grep -A20 -i "$1")
	if ! [[ -z "$STATUSRETVAL" ]];
	then
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
		if [ "$user_cmd" == "y" ]
		then
			echo "Continuing build with manually overridden submodule.."
		else
			if [ "$user_cmd" == "u" ]
			then
				git submodule sync --recursive
				git submodule update --init --recursive
				echo "Submodule fixed, continuing build.."
			else
				echo "Build aborted."
				exit 1
			fi
		fi
	fi
else
	git submodule sync --recursive;
	git submodule update --init --recursive $1;
fi

}

check_git_submodule NuttX
check_git_submodule Tools/gencpp
check_git_submodule Tools/genmsg
check_git_submodule Tools/jMAVSim
check_git_submodule Tools/sitl_gazebo
check_git_submodule cmake/cmake_hexagon
check_git_submodule mavlink/include/mavlink/v1.0
check_git_submodule src/lib/DriverFramework
check_git_submodule src/lib/DriverFramework/cmake/cmake_hexagon
check_git_submodule src/lib/DriverFramework/dspal
check_git_submodule src/lib/ecl
check_git_submodule src/lib/matrix
check_git_submodule src/modules/uavcan/libuavcan
check_git_submodule unittests/googletest

exit 0

