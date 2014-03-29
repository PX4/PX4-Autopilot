#!/bin/sh

num_cores=$(grep -c ^processor /proc/cpuinfo)
if [ -z "$num_cores" ]; then
    echo "num_cores=? WTF?"
    num_cores=4
fi

cppcheck . --error-exitcode=1 --quiet --enable=all --platform=unix64 --std=c99 --std=c++11 \
           --inline-suppr --force --template=gcc -j$num_cores \
           -Iinclude $@
