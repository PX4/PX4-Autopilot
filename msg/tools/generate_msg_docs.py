#!/usr/bin/env python3

"""
Generate docs from .msg files
"""

import os
import argparse
import sys


def get_msgs_list(msgdir):
    """
    Makes list of msg files in the given directory
    """
    return [fn for fn in os.listdir(msgdir) if fn.endswith(".msg")]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate docs from .msg files')
    parser.add_argument('-d', dest='dir', help='output directory', required=True)
    args = parser.parse_args()

    output_dir = args.dir
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    msg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"..")
    msg_files = get_msgs_list(msg_path)
    for msg_file in msg_files:
        msg_name = os.path.splitext(msg_file)[0]
        output_file = os.path.join(output_dir, msg_name+'.md')
        msg_filename = os.path.join(msg_path, msg_file)
        print("{:} -> {:}".format(msg_filename, output_file))

