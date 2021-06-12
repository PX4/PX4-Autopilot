#!/bin/bash

STYLE="google"

if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <src_file | dir>"
    echo ""
    echo "ERROR: At least one source file or one directory must be provided!"

    exit 1
fi

for arg in "$@"
do
    if [ -f $arg ]; then
        clang-format-6.0 -i -style='{BasedOnStyle: google, ColumnLimit: 120}' $arg
    elif [ -d $arg ]; then
        find $arg -iname '*.h' -o -iname '*.cpp' -o -iname '*.hpp' | xargs clang-format-6.0 -i -style='{BasedOnStyle: google, ColumnLimit: 120}'
        find $arg -iname '*.h' -o -iname '*.cpp' -o -iname '*.hpp' | xargs chmod 644
    fi
done
