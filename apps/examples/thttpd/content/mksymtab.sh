#!/bin/bash

usage="Usage: %0 <test-dir-path>"

dir=$1
if [ -z "$dir" ]; then
	echo "ERROR: Missing <test-dir-path>"
	echo ""
	echo $usage
	exit 1
fi

if [ ! -d "$dir" ]; then
	echo "ERROR: Directory $dir does not exist"
	echo ""
	echo $usage
	exit 1
fi

varlist=`find $dir -name "*-thunk.S"| xargs grep -h asciz | cut -f3 | sort | uniq`

echo "#ifndef __EXAMPLES_NXFLAT_TESTS_SYMTAB_H"
echo "#define __EXAMPLES_NXFLAT_TESTS_SYMTAB_H"
echo ""
echo "#include <nuttx/binfmt/symtab.h>"
echo ""
echo "static const struct symtab_s exports[] = "
echo "{"

for string in $varlist; do
	var=`echo $string | sed -e "s/\"//g"`
	echo "  {$string, $var},"
done

echo "};"
echo "#define NEXPORTS (sizeof(exports)/sizeof(struct symtab_s))"
echo ""
echo "#endif /* __EXAMPLES_NXFLAT_TESTS_SYMTAB_H */"

