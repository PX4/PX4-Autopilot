#!/usr/bin/env python
"""
The module facilitates testing in cmake.
It  takes a command and a regex for failure ok passing.
It passes if:
    * No stderr output.
    * Stdout doesn't match failure regex.
    * Stdout matches ok regex if given.
"""
from __future__ import print_function
import argparse
import subprocess
import re
import sys

#pylint: disable=invalid-name

parser = argparse.ArgumentParser()

parser.add_argument('cmd')
parser.add_argument('--re-fail')
parser.add_argument('--re-ok')
parser.add_argument('--verbose', '-v', dest='verbose', action='store_true')

parser.set_defaults(verbose=False)
args = parser.parse_args()

proc = subprocess.Popen(args.cmd.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
stdout, stderr = proc.communicate()

if stderr != "":
    print(stderr)
    sys.exit(1)

if args.re_fail is not None:
    fail_match = re.search(args.re_fail, stdout)
    if fail_match is not None:
        print(stdout)
        sys.exit(1)

if args.re_ok is not None:
    ok_match = re.search(args.re_ok, stdout)
    if re.match(args.re_ok, stdout) is None:
        print(stdout)
        sys.exit(1)

if args.verbose:
    print(stdout)

sys.exit(0)

# vim: set et ft=python fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :
