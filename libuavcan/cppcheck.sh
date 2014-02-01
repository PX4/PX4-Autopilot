#!/bin/sh
#
# cppcheck static analysis
# For Debian based: apt-get install cppcheck
#

# TODO: with future versions of cppcheck, add --library=glibc
cppcheck . --error-exitcode=1 --quiet --enable=all --platform=unix64 --std=c99 --std=c++11 \
           --inconclusive --inline-suppr --force --template=gcc \
           -Iinclude $@
