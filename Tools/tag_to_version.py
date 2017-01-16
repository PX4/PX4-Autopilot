#!/usr/bin/env python

import subprocess
import os
import argparse

p = argparse.ArgumentParser('finds major minor patch version from git tag')
p.add_argument('--root', help="root of git repo", default=".")
args = p.parse_args()
os.chdir(args.root)
p= subprocess.Popen(
    'git describe --always --tags'.split(),
    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
stdout, stderr = p.communicate()
res = stdout.split('-')[0].split('.')
major = res[0].replace('v','')
minor = res[1]
patch = res[2]
print("{:s}.{:s}.{:s}".format(major, minor, patch))
