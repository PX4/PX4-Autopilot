#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# diffconfig - a tool to compare .config files.
#
# originally written in 2006 by Matt Mackall
#  (at least, this was in his bloatwatch source code)
# last worked on 2008 by Tim Bird
#
# Source https://github.com/zephyrproject-rtos/zephyr/blob/zephyr-v2.3.0/scripts/kconfig/diffconfig
#
# Modified to include negated values during merge_style

import sys, os

def usage():
    print("""Usage: diffconfig [-h] [-m] [<config1> <config2> [<resolved>]]

Diffconfig is a simple utility for comparing two .config files.
Using standard diff to compare .config files often includes extraneous and
distracting information.  This utility produces sorted output with only the
changes in configuration values between the two files.

Added and removed items are shown with a leading plus or minus, respectively.
Changed items show the old and new values on a single line.

If -m is specified, then output will be in "merge" style, which has the
changed and new values in kernel config option format.

If no config files are specified, .config and .config.old are used.

PX4: if <resolved> is given, the full resolved config <config2> was saved
from, an item missing from <config2> is not taken as n: with -m it is
written with its resolved value, and only if that differs from <config1>.
A minimal config leaves out every item that has its default value, which
can be y through an imply, select or default.

Example usage:
 $ diffconfig .config config-with-some-changes
-EXT2_FS_XATTR  n
-EXT2_FS_XIP  n
 CRAMFS  n -> y
 EXT2_FS  y -> n
 LOG_BUF_SHIFT  14 -> 16
 PRINTK_TIME  n -> y
""")
    sys.exit(0)

# returns a dictionary of name/value pairs for config items in the file
def readconfig(config_file):
    d = {}
    for line in config_file:
        line = line[:-1]
        if line[:7] == "CONFIG_":
            name, val = line[7:].split("=", 1)
            d[name] = val
        if line[-11:] == " is not set":
            d[line[9:-11]] = "n"
    return d

def print_config(op, config, value, new_value):
    global merge_style

    if merge_style:
        if op=="-" and value=="y":
            print("CONFIG_%s=n" % (config))
        elif op=="-" and not new_value:
            print("# CONFIG_%s is not set" % (config))
        elif new_value:
            if new_value=="n":
                print("# CONFIG_%s is not set" % config)
            else:
                print("CONFIG_%s=%s" % (config, new_value))
    else:
        if op=="-":
            print("-%s %s" % (config, value))
        elif op=="+":
            print("+%s %s" % (config, new_value))
        else:
            print(" %s %s -> %s" % (config, value, new_value))

def main(merge, configa_filename, configb_filename, resolved_filename=None):
    global merge_style

    merge_style = merge

    try:
        a = readconfig(open(configa_filename))
        b = readconfig(open(configb_filename))
        resolved = readconfig(open(resolved_filename)) if resolved_filename else None
    except (IOError):
        e = sys.exc_info()[1]
        print("I/O error[%s]: %s\n" % (e.args[0],e.args[1]))
        usage()

    # print items in a but not b (accumulate, sort and print)
    old = []
    for config in a:
        if config not in b:
            old.append(config)
    old.sort()
    for config in old:
        if resolved is None:
            print_config("-", config, a[config], None)
        elif resolved.get(config, "n") != a[config]:
            # PX4: write the resolved value; n is not implied by a missing item
            if merge_style:
                print("CONFIG_%s=%s" % (config, resolved.get(config, "n")))
            else:
                print_config("->", config, a[config], resolved.get(config, "n"))
        del a[config]

    # print items that changed (accumulate, sort, and print)
    changed = []
    for config in a:
        if a[config] != b[config]:
            changed.append(config)
        else:
            del b[config]
    changed.sort()
    for config in changed:
        print_config("->", config, a[config], b[config])
        del b[config]

    # now print items in b but not in a
    # (items from b that were in a were removed above)
    new = sorted(b.keys())
    for config in new:
        print_config("+", config, None, b[config])

if __name__ == '__main__':

    # parse command line args
    if ("-h" in sys.argv or "--help" in sys.argv):
        usage()

    merge_style = 0
    if "-m" in sys.argv:
        merge_style = 1
        sys.argv.remove("-m")

    argc = len(sys.argv)
    if not (argc == 1 or argc == 3 or argc == 4):
        print("Error: incorrect number of arguments or unrecognized option")
        usage()

    if argc == 1:
        # if no filenames given, assume .config and .config.old
        build_dir=""
        if "KBUILD_OUTPUT" in os.environ:
            build_dir = os.environ["KBUILD_OUTPUT"]+"/"
        configa_filename = build_dir + ".config.old"
        configb_filename = build_dir + ".config"
        resolved_filename = None
    else:
        configa_filename = sys.argv[1]
        configb_filename = sys.argv[2]
        resolved_filename = sys.argv[3] if argc == 4 else None

    main(merge_style, configa_filename, configb_filename, resolved_filename)
