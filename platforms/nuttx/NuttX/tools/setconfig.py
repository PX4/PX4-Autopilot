#!/usr/bin/env python

# Copyright (c) 2019, Ulf Magnusson
# SPDX-License-Identifier: ISC

import argparse
import sys

import kconfiglib


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="""
Simple utility for setting configuration values from the command line.

Sample usage:

  $ setconfig FOO_SUPPORT=y BAR_BITS=8

Note: Symbol names should not be prefixed with 'CONFIG_'.

The exit status on errors is 1.

The default input/output configuration file is '.config'. A different filename
can be passed in the KCONFIG_CONFIG environment variable.
""")

    parser.add_argument(
        "--kconfig",
        default="Kconfig",
        help="Base Kconfig file (default: Kconfig)")

    parser.add_argument(
        "--no-check-exists",
        dest='check_exists',
        action='store_false',
        help="Ignore assignments to non-existent symbols instead of erroring "
             "out")

    parser.add_argument(
        "--no-check-value",
        dest='check_value',
        action='store_false',
        help="Ignore assignments that didn't \"take\" (where the symbol got a "
             "different value, e.g. due to unsatisfied dependencies) instead "
             "of erroring out")

    parser.add_argument(
        "assignments",
        metavar="ASSIGNMENT",
        nargs="*",
        help="A 'NAME=value' assignment")

    args = parser.parse_args()

    kconf = kconfiglib.Kconfig(args.kconfig)
    kconf.load_config()

    for arg in args.assignments:
        if "=" not in arg:
            sys.exit("error: no '=' in assignment: '{}'".format(arg))
        name, value = arg.split("=", 1)

        if name not in kconf.syms:
            if not args.check_exists:
                continue
            sys.exit("error: no symbol '{}' in configuration".format(name))

        sym = kconf.syms[name]

        if not sym.set_value(value):
            sys.exit("error: '{}' is an invalid value for the {} symbol {}"
                     .format(value, kconfiglib.TYPE_TO_STR[sym.type], name))

        if args.check_value and sym.str_value != value:
            sys.exit("error: {} was assigned the value '{}', but got the "
                     "value '{}'. Check the symbol's dependencies, and make "
                     "sure that it has a prompt."
                     .format(name, value, sym.str_value))

    kconf.write_config()


if __name__ == "__main__":
    main()
