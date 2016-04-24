#!/usr/bin/env bash

file=$1

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

if [ -f "$file" ];
then
	${DIR}/fix_code_style.sh --dry-run $file | grep --quiet Formatted
	if [[ $? -eq 0 ]]
	then
		echo $file 'bad formatting, please run "./Tools/fix_code_style.sh' $file'"'
		exit 1
	fi
fi

