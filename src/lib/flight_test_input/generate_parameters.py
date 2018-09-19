#! /usr/bin/env python
""" Script to generate flight test input parameters """

from __future__ import print_function

import argparse
import os
import sys

from jinja2 import Environment, FileSystemLoader

parser = argparse.ArgumentParser(description='Generate Flight Test Input parameters')

parser.add_argument('--groups', type=str, nargs='*', metavar="TAG:DEVICE",
                    default=[],
                    help='Flight Test Input parameter group (eg PWM1, PWM2, GPS)')
parser.add_argument('--params-file', type=str, action='store',
                    help='Parameter output file', default=None)
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()

arg_groups = args.groups
verbose = args.verbose
flight_test_input_params_output_file = args.params_file
flight_test_input_params_template = 'flight_test_input_params.c.jinja'

jinja_env = Environment(loader=FileSystemLoader(
    os.path.dirname(os.path.realpath(__file__))))

# parameter definitions
if flight_test_input_params_output_file is not None:
    if verbose: print("Generating {:}".format(flight_test_input_params_output_file))
    template = jinja_env.get_template(flight_test_input_params_template)
    with open(flight_test_input_params_output_file, 'w') as fid:
        fid.write(template.render(param_groups=arg_groups))

