#!/usr/bin/env bash
set -eu

ASTYLE_VER_REQUIRED="Artistic Style Version 2.06"
astyle_ver() {
	echo "PX4 requires ${ASTYLE_VER_REQUIRED}"
	echo "You can get the correct version here: https://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/"
}

# check if astyle is installed
condition=$(which astyle 2>/dev/null | grep -v "not found" | wc -l)
if [ $condition -eq 0 ]; then
	echo "astyle is not installed"
	astyle_ver
	exit 1
else
	ASTYLE_VER=`astyle --version`

	if [ "$ASTYLE_VER" != "$ASTYLE_VER_REQUIRED" ]; then
		echo "Error: you're using ${ASTYLE_VER}"
		astyle_ver
		exit 1
	fi
fi

CI="${CI:-false}"
DIR=$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)

if [[ "$@" == "--fix" ]]
then
    export PX4_ASTYLE_FIX=1
fi


# install git pre-commit hook
HOOK_FILE="$DIR/../.git/hooks/pre-commit"
if [ ! -f $HOOK_FILE ] && [ "$CI" != "true" ]; then
	echo ""
	echo -e "\033[31mNinja tip: add a git pre-commit hook to automatically check code style\033[0m"
	echo -e "Would you like to install one now? (\033[94mcp ./Tools/pre-commit .git/hooks/pre-commit\033[0m): [y/\033[1mN\033[0m]"

	read user_cmd
	if [ "$user_cmd" == "y" ]; then
		echo -e "copying ./Tools/pre-commit -> .git/hooks/pre-commit"
		mkdir -p $DIR/../.git/hooks
		cp $DIR/pre-commit $HOOK_FILE
		echo -e "\033[94mGreat, hook installed!\033[0m (checking style now)"
	else
		echo -e "\033[94mOk, I will remind you again later!\033[0m (checking style now)"
	fi
fi

${DIR}/files_to_check_code_style.sh | xargs -n 1 -P 8 -I % ${DIR}/check_code_style.sh %

if [ $? -eq 0 ]; then
    echo "Format checks passed"
    exit 0
fi
