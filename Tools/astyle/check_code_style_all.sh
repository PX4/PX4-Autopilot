#!/usr/bin/env bash
set -eu

DIR=$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)

# fail early if no usable astyle is installed
"${DIR}/fix_code_style.sh"

CI="${CI:-false}"

DIFF_ONLY=false
for arg in "$@"; do
    case "$arg" in
        --fix) export PX4_ASTYLE_FIX=1 ;;
        --diff-only) DIFF_ONLY=true ;;
    esac
done


# install git pre-commit hook
HOOK_FILE="$DIR/../../.git/hooks/pre-commit"
if [[ ! -f "$HOOK_FILE" && "$CI" != "true" && $- == *i* ]]; then
	echo ""
	echo -e "\033[31mNinja tip: add a git pre-commit hook to automatically check code style\033[0m"
	echo -e "Would you like to install one now? (\033[94mcp ./Tools/astyle/pre-commit .git/hooks/pre-commit\033[0m): [y/\033[1mN\033[0m]"

	read user_cmd
	if [ "$user_cmd" == "y" ]; then
		echo -e "copying ./Tools/astyle/pre-commit -> .git/hooks/pre-commit"
		mkdir -p $DIR/../.git/hooks
		cp $DIR/pre-commit $HOOK_FILE
		echo -e "\033[94mGreat, hook installed!\033[0m (checking style now)"
	else
		echo -e "\033[94mOk, I will remind you again later!\033[0m (checking style now)"
	fi
fi

FILE_LIST=$(${DIR}/files_to_check_code_style.sh)

if [ "$DIFF_ONLY" = true ]; then
    # --diff-filter=d: do not list deleted files (no need to format)
    CHANGED=$(git -C "$DIR/../.." diff --name-only --diff-filter=d HEAD -- '*.c' '*.h' '*.cpp' '*.hpp')
    if [ -z "$CHANGED" ]; then
        FILE_LIST=""
    else
        FILE_LIST=$(echo "$FILE_LIST" | grep -Fx -f <(echo "$CHANGED") || true)
    fi
fi

if [ -z "$FILE_LIST" ]; then
    echo "No files to check"
    exit 0
fi

echo "$FILE_LIST" | xargs -P 8 -n 100 "${DIR}/check_code_style.sh"

if [ $? -eq 0 ]; then
    echo "Format checks passed"
    exit 0
fi
