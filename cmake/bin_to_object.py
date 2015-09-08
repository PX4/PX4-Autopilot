#!/usr/bin/env python
"""
This converts a binary imagge to an object file
"""
from __future__ import print_function
import subprocess
import argparse
import os

#pylint: disable=invalid-name
parser = argparse.ArgumentParser(description='Convert bin to obj.')
parser.add_argument('--c-flags', required=True)
parser.add_argument('--c-compiler', required=True)
parser.add_argument('--nm', required=True)
args = parser.parse_args()

#TODO write function

exit(0)

# vim: set et ft=python fenc= ff=unix sts=4 sw=4 ts=4 :
