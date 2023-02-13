#!/usr/bin/env python

import argparse
import lzma

parser = argparse.ArgumentParser(description="""Compress a file with xz""")
parser.add_argument('filename', metavar='file', help='Input file (output: file.xz)')

args = parser.parse_args()
filename = args.filename

def save_compressed(filename):
    #create xz compressed version
    xz_filename=filename+'.xz'
    with lzma.open(xz_filename, 'wt', preset=9) as f:
        with open(filename, 'r') as content_file:
            f.write(content_file.read())

save_compressed(filename)
