#!/bin/bash
echo pwd:$PWD
astyle=$1
format=$2
format_wildcards="""
./matrix/*.*pp
./test/*.*pp
"""

#echo astyle: $astyle
#echo format: $format

if [[ $format -eq 1 ]]
then
	echo formatting
	$astyle ${format_wildcards}
else
	echo checking format
	$astyle --dry-run ${format_wildcards} | grep Formatted
	if [[ $? -eq 0 ]]
	then
		echo need to format
		exit 1
	fi
	echo no formatting needed
	exit 0
fi
