#!/usr/bin/env python
"""
This runs a command and compares output to a known file over
a given line range.
"""
from __future__ import print_function
import subprocess
import argparse
import os


#pylint: disable=invalid-name
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--command', required=True)
parser.add_argument('--stdin', required=True)
parser.add_argument('--stdout', required=True)
parser.add_argument('--check', required=True)
parser.add_argument('--start', default=0)
parser.add_argument('--stop', default=-1)
args = parser.parse_args()

d = os.path.dirname(args.stdout)
if not os.path.exists(d):
    os.makedirs(d)

with open(args.stdout, 'w') as outfile:
    with open(args.stdin, 'r') as infile:
        proc = subprocess.Popen(
            args.command, stdout=outfile, stdin=infile)
proc.communicate()

i_start = int(args.start)
i_stop = int(args.stop)

with open(args.stdout, 'r') as outfile:
    out_contents = file.readlines(outfile)
out_contents = "".join(out_contents[i_start:i_stop])

with open(args.check, 'r') as checkfile:
    check_contents = file.readlines(checkfile)
check_contents = "".join(check_contents[i_start:i_stop])

if (out_contents != check_contents):
    print("output:\n", out_contents)
    print("check:\n", check_contents)
    exit(1)

exit(0)

# vim: set et ft=python fenc= ff=unix sts=4 sw=4 ts=4 :
