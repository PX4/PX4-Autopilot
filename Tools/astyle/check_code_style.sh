#!/usr/bin/env bash

FILE=$1
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

CHECK_FAILED=$(${DIR}/fix_code_style.sh --dry-run --formatted $FILE)
if [ -n "$CHECK_FAILED" ]; then
	${DIR}/fix_code_style.sh --quiet < $FILE > $FILE.pretty

	echo -e 'Formatting issue found in' $FILE
	echo
	git --no-pager diff --no-index --minimal --histogram --color=always $FILE $FILE.pretty | grep -vE -e "^.{,4}diff.*\.pretty.{,3}$" -e "^.{,4}--- a/.*$" -e "^.{,4}\+\+\+ b/.*$" -e "^.{,5}@@ .* @@.*$" -e "^.{,4}index .{10}\.\."
	rm -f $FILE.pretty
	echo

	if [[ $PX4_ASTYLE_FIX -eq 1 ]]; then
		${DIR}/fix_code_style.sh $FILE
	else
		echo 'to fix automatically run "make format" or "./Tools/astyle/fix_code_style.sh' $FILE'"'
		exit 1
	fi
fi
