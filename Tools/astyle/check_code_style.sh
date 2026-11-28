#!/usr/bin/env bash
# Check the formatting of the given files, print a diff for each badly formatted
# one and fail. Never modifies the files unless --fix is given, which writes the
# formatted version back after printing the diff.
#
# All files are formatted in one batch on a temporary copy to keep the
# formatter startup cost independent of the number of files.

FIX=false
if [[ "$1" == "--fix" ]]; then
	FIX=true
	shift
fi

if [[ $# -eq 0 ]]; then
	exit 0
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT
tar cf - -- "$@" | tar xf - -C "$TMP_DIR" || exit 1

FORMATTED=()
for FILE in "$@"; do
	FORMATTED+=("$TMP_DIR/$FILE")
done

"${DIR}/fix_code_style.sh" "${FORMATTED[@]}" || exit 1

ISSUES=false
for FILE in "$@"; do
	cmp -s "$FILE" "$TMP_DIR/$FILE" && continue
	ISSUES=true
	echo "Formatting issue found in $FILE"
	echo
	git --no-pager diff --no-index --minimal --histogram --color=always "$FILE" "$TMP_DIR/$FILE" | tail -n +5
	echo
	if $FIX; then
		cat "$TMP_DIR/$FILE" > "$FILE"
	fi
done

if $ISSUES && ! $FIX; then
	echo 'to fix automatically run "make format" or "make format_changed"'
	exit 1
fi
