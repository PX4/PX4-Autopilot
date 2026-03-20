#!/usr/bin/env python3

import argparse
import em
import os
import sys

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument("-t", "--template_file", dest='template_file', type=str,
                    help="empy template file")
parser.add_argument("-o", "--output", dest='output_file', type=str,
                    help="output filename", default=None)
parser.add_argument("-p", "--prefix_name", dest='prefix_name', type=str,
                    help="Package prefix name")

if len(sys.argv) <= 1:
    parser.print_usage()
    exit(-1)

# Parse arguments
args = parser.parse_args()

template_file = os.path.join(args.template_file)
if(args.output_file):
    o_file = args.output_file
else:
    o_file = os.path.splitext(template_file)[0]

template_defines = {}
template_defines['PREFIX'] = args.prefix_name

# run interpreter
ofile = open(o_file, 'w')
interpreter = em.Interpreter(output=ofile, globals=template_defines)

try:
    interpreter.file(open(template_file))
finally:
    interpreter.shutdown()
    ofile.close()
