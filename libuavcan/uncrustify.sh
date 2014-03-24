#!/bin/sh

files="$files $(find include -name '*.hpp')"
files="$files $(find src -name '*.cpp')"

uncrustify --replace --no-backup -c uncrustify.cfg $files
