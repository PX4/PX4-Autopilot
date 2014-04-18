#!/bin/sh

#files="$files $(find include -name '*.hpp')"
#files="$files $(find src -name '*.cpp')"
#files="$files $(find test -name '*.cpp')"
#files="$files $(find test -name '*.hpp')"

uncrustify --replace --no-backup -c uncrustify.cfg $files
