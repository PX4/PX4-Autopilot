#!/usr/bin/env python3
""" Script to generate Serial rc.filepaths for the ROMFS startup script """

import argparse
import os
import sys

try:
    from jinja2 import Environment, FileSystemLoader
except ImportError as e:
    print("Failed to import jinja2: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user jinja2")
    print("")
    sys.exit(1)

try:
    import yaml
except ImportError as e:
    print("Failed to import yaml: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyyaml")
    print("")
    sys.exit(1)

parser = argparse.ArgumentParser(description='Generate PX4 ROMFS filepaths')

parser.add_argument('--config-files', type=str, nargs='*', default=[],
                    help='YAML module config file(s)')
parser.add_argument('--constrained-flash', action='store_true',
                    help='Reduce verbosity in ROMFS scripts to reduce flash size')
parser.add_argument('--rc-dir', type=str, action='store',
                    help='ROMFS output directory', default=None)
parser.add_argument('--params-file', type=str, action='store',
                    help='Parameter output file', default=None)
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()

verbose = args.verbose
constrained_flash = args.constrained_flash
rc_filepaths_output_dir = args.rc_dir
rc_filepaths_template = 'rc.filepaths.jinja'


jinja_env = Environment(loader=FileSystemLoader(
    os.path.dirname(os.path.realpath(__file__))))

# generate the ROMFS script using a jinja template
if rc_filepaths_output_dir is not None:
    rc_filepath_output_file = os.path.join(rc_filepaths_output_dir, "rc.filepaths")

    if verbose: print("Generating {:}".format(rc_filepath_output_file))
    template = jinja_env.get_template(rc_filepaths_template)
    with open(rc_filepath_output_file, 'w') as fid:
        fid.write(template.render(constrained_flash=constrained_flash, params_file=args.params_file))
else:
    raise Exception("--rc-dir needs to be specified")
