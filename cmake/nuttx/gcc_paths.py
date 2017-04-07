#!/usr/bin/env python
"""
This finds the appropriate library paths for cross-compiling using GNU Toolchain + Clang.
"""
from __future__ import print_function
import subprocess
import argparse
import re
import os
from collections import OrderedDict
from subprocess import PIPE

def print_lib_paths(cmd):
    cmd = cmd + " -print-search-dirs"
    proc = subprocess.Popen(cmd.split(), stdout=PIPE, stderr=PIPE)
    stdout, stderr = proc.communicate()
    s = re.search('libraries: =.\S+', stdout.decode()).group()
    s = s.replace('libraries: =', "")
    library_paths = s.split(':')
    library_paths = list(OrderedDict.fromkeys(library_paths))

    output = ""
    for path in library_paths:
        if (os.path.exists(path)):
            path = os.path.abspath(path)
            output += " " + path.strip()
    print(output, end='')

def print_inc_paths(cmd, cxx=False):
    if cxx:
        cmd += ' -xc++ '

    cmd = cmd + ' -v -E -nostdlib -nodefaultlibs - < /dev/null 2>&1 | grep -E "^ \S+$" | xargs -- readlink -f'
    proc = subprocess.Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)
    stdout, stderr = proc.communicate()
    s = stdout.decode()
    inc_paths = s.split('\n')
    inc_paths = list(OrderedDict.fromkeys(inc_paths))

    output = ""
    for path in inc_paths:
        if (os.path.exists(path)):
            path = os.path.abspath(path)
            output += " -isystem " + path.strip()

    print(output, end='')

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Convert bin to obj.')
    parser.add_argument('--c_flags', required=True)
    parser.add_argument('--c_compiler', required=True)
    parser.add_argument('--cxx', action="store_true")
    parser.add_argument('--lib_paths', action="store_true")
    parser.add_argument('--inc_paths', action="store_true")

    args = parser.parse_args()

    c_flags = args.c_flags
    c_compiler = args.c_compiler
    lib_paths = args.lib_paths
    inc_paths = args.inc_paths
    cxx = args.cxx

    cmd = "{c_compiler:s} {c_flags:s}".format(**locals())

    if lib_paths:
        print_lib_paths(cmd)
    elif inc_paths:
        print_inc_paths(cmd, cxx)
