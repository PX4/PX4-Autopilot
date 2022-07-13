#!/usr/bin/env python3
"""
Script to print out the board information JSON manifest

Example:
{
    "include":
        [
            {
                "target": "px4_fmu-v5x_default",
                "board_id": "51"
            },
        ...
        ]
}
"""

import argparse
import os
import json
import re
from kconfiglib import Kconfig

parser = argparse.ArgumentParser(description='Generate build targets')

parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')
parser.add_argument('-p', '--pretty', dest='pretty', action='store_true',
                    help='Pretty output instead of a single line')
args = parser.parse_args()

# GLOBAL VARIABLES
verbose = args.verbose
build_configs = []
excluded_manufacturers = ['atlflight']
excluded_platforms = ['qurt']
excluded_labels = [
    'stackcheck',
    'nolockstep', 'replay', 'test',
    'uavcanv1' # TODO: fix and enable
    ]

# Supress warning output while parsing KConfig
kconf = Kconfig()
kconf.warn_assign_undef = False
kconf.warn_assign_override = False
kconf.warn_assign_redun = False

''' Turn given firmware.prototype into a dictionary with board information '''
def process_target_firmware_prototype_file(manufacturer, board, file):
    if not (file.is_file() and file.name == 'firmware.prototype'):
        return None

    global verbose, excluded_labels
    target_name = manufacturer.name + '_' + board.name

    with open(file.path, 'r') as f:
        data = json.load(f)
        board_info_dict = dict()
        board_info_dict['board_name'] = data['summary']
        board_info_dict['target_name'] = target_name
        board_info_dict['description'] = data['description']
        board_info_dict['board_id'] = data['board_id']
        board_info_dict['image_max_size'] = data['image_maxsize']
        board_info_dict['build_variants'] = []

        return board_info_dict

'''
Process the firmware.prototype files and create board information Json file

In this following format:

{
    "board_name"        = "HUMAN_READABLE_BOARD_NAME",      (summary)
    "target_name"       = "TARGET_NAME_FOR_BUILD",          (manufacturer_board_label)
    "description"       = "DESCRIPTION",                    (description)
    "board_id"          = "BOOTLOADER_ID",
    "image_maxsize"     = "FLASH_SIZE",
    "build_variants"    = [ {"name": "VARIANT_1"}, ... ]    (labels)
}
'''
def print_board_information(board_list: list):
    global args, verbose
    board_info_list = []        # Board information list

    for board in board_list:
        board_info = dict()     # Board information that will be extracted from firmware.prototype
        board_variants = []     # Save different board variants (labels)

        for file in board['files']:

            # Process all the board variants
            if file.name.endswith(".px4board"):
                label = file.name.split(".px4board")[0]
                board_variants.append({"name": label}) # For now we only have the 'name' attribute (e.g. default, rtps, etc.)

            # Process the firwmare.prototype
            elif file.name == "firmware.prototype":
                board_info = process_target_firmware_prototype_file(board['manufacturer'], board['board'], file)

        if board_info is None and verbose:
            print("Error, didn't detect valid firmware.prototype for the board. This board won't be added to the Board information JSON file! : {} - {}".format(board['manufacturer'], board['file']))

        if board_info is not None:
            board_info['build_variants'] = board_variants
            board_info_list.append(board_info)

    # Final dictionary that will be turned into the JSON file
    board_info_dict = { 'board_info' : board_info_list }

    extra_args = dict()         # Extra arg that will be added to the JSON
    if args.pretty:
        extra_args['indent'] = 2

    print(json.dumps(board_info_dict, **extra_args))


def main():
    global verbose, build_configs
    source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

    board_list = []
    for manufacturer in os.scandir(os.path.join(source_dir, 'boards')):
        if not manufacturer.is_dir():
            continue
        if manufacturer.name in excluded_manufacturers:
            if verbose: print(f'excluding manufacturer {manufacturer.name}')
            continue

        for board in os.scandir(manufacturer.path):
            if not board.is_dir():
                continue

            file_list = [] # Group the files under same board together
            for file in os.scandir(board.path):
                if not file.is_dir():
                    file_list.append(file)

            board_list.append({'manufacturer': manufacturer, 'board' : board, 'files' : file_list})

    print_board_information(board_list)

if __name__ == '__main__':
    main()
