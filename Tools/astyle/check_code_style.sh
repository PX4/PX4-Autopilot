#!/usr/bin/env bash
# Check the formatting of the given files, print a diff for each badly formatted
# one and fail. With PX4_ASTYLE_FIX=1 the files get fixed instead.
#
# All files are formatted in one batch on a temporary copy to keep the
# formatter startup cost independent of the number of files.

if [[ $# -eq 0 ]]; then
	exit 0
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT

FORMATTED=()
for FILE in "$@"; do
	mkdir -p "$TMP_DIR/$(dirname "$FILE")"
	cp "$FILE" "$TMP_DIR/$FILE"
	FORMATTED+=("$TMP_DIR/$FILE")
done

"${DIR}/fix_code_style.sh" "${FORMATTED[@]}" || exit 1

CHECK_FAILED=0
for FILE in "$@"; do
	if ! cmp -s "$FILE" "$TMP_DIR/$FILE"; then
		echo -e 'Formatting issue found in' "$FILE"
		echo
		git --no-pager diff --no-index --minimal --histogram --color=always "$FILE" "$TMP_DIR/$FILE" | tail -n +5
		echo

		if [[ $PX4_ASTYLE_FIX -eq 1 ]]; then
			cp "$TMP_DIR/$FILE" "$FILE"
		else
			echo 'to fix automatically run "make format", "make format_changed" or "./Tools/astyle/fix_code_style.sh' "$FILE"'"'
			CHECK_FAILED=1
		fi
	fi
done

exit $CHECK_FAILED
