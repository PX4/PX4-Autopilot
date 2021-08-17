#!/bin/python3

import parse_cmake.parsing as cmp
import glob
import pprint
import re
import os

__location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))

serial_regex = r"(\D\D\D\d):(/dev/ttyS\d+)"
io_regex = r"IO (.*)"
romfs_regex = r"ROMFSROOT (.*)"



def stripComments(code):
    code = str(code)
    return re.sub(r'(?m) *#.*\n?', '', code)

lut = {}
with open(os.path.join(__location__, "cmake_kconfig_lut.txt"),'r') as lookup:
    for line in lookup:
        if ',' in line:
            key, value = line.strip().split(',')
            lut[key] = value

#for name in glob.glob('boards/*/*/*.cmake'):
px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))

for name in glob.glob(px4_dir + '/boards/*/*/*.cmake'):
    print(name)
    with open(name, 'r') as f:
        romfs_set = False
        w = open(name.replace(".cmake",".px4board"), "w")
        for line in f:
            clean_line = stripComments(line.strip())
            value = lut.get(clean_line)
            if value is not None:
                print(value, file=w)
                print(value)
            else:
                matches = re.finditer(serial_regex, clean_line, re.MULTILINE)
                for matchNum, match in enumerate(matches, start=1):
                    print("CONFIG_BOARD_SERIAL_" + match.groups()[0] + "=\"" + match.groups()[1] + "\"")
                    print("CONFIG_BOARD_SERIAL_" + match.groups()[0] + "=\"" + match.groups()[1] + "\"", file=w)
                matches = re.finditer(io_regex, clean_line, re.MULTILINE)
                for matchNum, match in enumerate(matches, start=1):
                    print("CONFIG_BOARD_IO=\"" + match.groups()[0] + "\"")
                    print("CONFIG_BOARD_IO=\"" + match.groups()[0] + "\"", file=w)
                matches = re.finditer(romfs_regex, clean_line, re.MULTILINE)
                for matchNum, match in enumerate(matches, start=1):
                    print("CONFIG_BOARD_ROMFSROOT=\"" + match.groups()[0] + "\"")
                    print("CONFIG_BOARD_ROMFSROOT=\"" + match.groups()[0] + "\"", file=w)
                    romfs_set = True

    if(romfs_set == False):
        print("CONFIG_BOARD_ROMFSROOT=\"\"", file=w)


    w.close()
