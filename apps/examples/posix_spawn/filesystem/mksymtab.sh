#!/bin/bash

usage="Usage: $0 <test-dir-path>"

# Check for the required ROMFS directory path

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

# Extract all of the undefined symbols from the ELF files and create a
# list of sorted, unique undefined variable names.

varlist=`find ${dir} -executable -type f | xargs nm | fgrep ' U ' | sed -e "s/^[ ]*//g" | cut -d' ' -f2 | sort | uniq`

# Now output the symbol table as a structure in a C source file.  All
# undefined symbols are declared as void* types.  If the toolchain does
# any kind of checking for function vs. data objects, then this could
# faile

echo "#include <nuttx/compiler.h>"
echo "#include <nuttx/binfmt/symtab.h>"
echo ""

for var in $varlist; do
	echo "extern void *${var};"
done

echo ""
echo "const struct symtab_s exports[] = "
echo "{"

for var in $varlist; do
	echo "  {\"${var}\", &${var}},"
done

echo "};"
echo ""
echo "const int nexports = sizeof(exports) / sizeof(struct symtab_s);"

