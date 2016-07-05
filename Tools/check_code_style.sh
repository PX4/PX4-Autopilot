#!/usr/bin/env bash

file=$1

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

if [ -f "$file" ];
then
	${DIR}/fix_code_style.sh --dry-run $file | grep --quiet Formatted
	if [[ $? -eq 0 ]]
	then
		${DIR}/fix_code_style.sh --quiet < $file > $file.pretty

		echo
		git --no-pager diff --no-index --minimal --histogram --color=always  $file $file.pretty
		echo

		rm -f $file.pretty
		echo $file 'bad formatting, please run "./Tools/fix_code_style.sh' $file'"'
		exit 1
	fi
fi

