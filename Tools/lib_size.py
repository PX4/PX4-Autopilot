#!/usr/bin/env python
import fnmatch
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("path")
args = parser.parse_args()

libs = []
for root, dirnames, filenames in os.walk(args.path):
    for filename in fnmatch.filter(filenames, '*.a'):
        fn = os.path.join(root, filename)
        libs.append((fn, os.path.getsize(fn)))

for lib in sorted(libs, key=lambda t: t[1], reverse=True):
    path = lib[0]
    parent_dir = path.split(os.path.sep)[1]
    name = os.path.basename(path)
    size = lib[1]/1024.0/1024.0
    if int(size) > 0:
        print('{:30s} : {:40s}\tsize:\t{:5.2f}\tMB'.format(parent_dir, name, size))


