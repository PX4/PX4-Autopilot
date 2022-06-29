#!/usr/bin/env python3
"""
Script to generate a JSON config with all build targets (for CI), in 2 different versions:

1. For enumerating targets: {"include":[{"target": "px4_fmu-v5x_default", "container": "nuttx-focal"}, ...]}

Called by : Tools/generate_board_targets_json.py -e

---

2. For board information: {"include":[{"target": "px4_fmu-v5x_default", "board_id": "51"}, ...]}

Called by : Tools/generate_board_targets_json.py -b

"""

import argparse
import os
import json
import re
from kconfiglib import Kconfig

parser = argparse.ArgumentParser(description='Generate build targets')

parser.add_argument('-e', '--enumerate', dest='enumerate', action='store_true',
                    help='Output enumerated targets')
parser.add_argument('-b', '--boardinfo', dest='boardinfo', action='store_true',
                    help='Output target board information')
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')
parser.add_argument('-p', '--pretty', dest='pretty', action='store_true',
                    help='Pretty output instead of a single line')
args = parser.parse_args()

if not args.enumerate and not args.boardinfo:
    print("Please either specify -e or -v flag to indicate which format to output!")
    exit()

# GLOBAL VARIABLES
verbose = args.verbose
enumerate_mode = args.enumerate
boardinfo_mode = args.boardinfo

build_configs = []
excluded_manufacturers = ['atlflight']
excluded_platforms = ['qurt']
excluded_labels = [
    'stackcheck',
    'nolockstep', 'replay', 'test',
    'uavcanv1' # TODO: fix and enable
    ]


kconf = Kconfig()

# Supress warning output
kconf.warn_assign_undef = False
kconf.warn_assign_override = False
kconf.warn_assign_redun = False


''' Turn given file (.px4board) into a dictionary with board and toolchain information'''
def process_target_px4board_file(manufacturer, board, file):
    global verbose, excluded_labels

    ret = None
    platform = None
    toolchain = None

    if not(file.is_file() and file.name.endswith('.px4board')):
        return None

    label = file.name.split(".px4board")[0]
    target_name = manufacturer.name + '_' + board.name + '_' + label

    if label in excluded_labels:
        if verbose: print(f'excluding label {label} ({target_name})')
        return None

    px4board_file = file.path

    if px4board_file.endswith("default.px4board") or \
        px4board_file.endswith("recovery.px4board") or \
        px4board_file.endswith("bootloader.px4board"):
        kconf.load_config(px4board_file, replace=True)

    else: # Merge config with default.px4board
        default_kconfig = re.sub(r'[a-zA-Z\d_]+\.px4board', 'default.px4board', px4board_file)
        kconf.load_config(default_kconfig, replace=True)
        kconf.load_config(px4board_file, replace=False)

    if "BOARD_TOOLCHAIN" in kconf.syms:
        toolchain = kconf.syms["BOARD_TOOLCHAIN"].str_value

    if "BOARD_PLATFORM" in kconf.syms:
        platform = kconf.syms["BOARD_PLATFORM"].str_value

    assert platform, f"PLATFORM not found in {px4board_file}"

    if platform not in excluded_platforms:
        # get the container based on the platform and toolchain
        container = platform
        if platform == 'posix':
            container = 'base-focal'
            if toolchain:
                if toolchain.startswith('aarch64'):
                    container = 'aarch64'
                elif toolchain == 'arm-linux-gnueabihf':
                    container = 'armhf'
                else:
                    if verbose: print(f'possibly unmatched toolchain: {toolchain}')
        elif platform == 'nuttx':
            container = 'nuttx-focal'

        ret = {'target': target_name, 'container': container}

    return ret

'''
Process the given board files list and print boards list Json file
'''
def print_board_enumeration(board_list: list):
    global args
    extra_args = dict()

    json_list = []
    for board in board_list:
        for file in board['files']:
            target = process_target_px4board_file(board['manufacturer'], board['board'], file)
            if target is not None:
                json_list.append(target)

    github_action_config = { 'include' : json_list }

    if args.pretty:
        extra_args['indent'] = 2
    print(json.dumps(github_action_config, **extra_args))



''' Turn given firmware.prototype into a dictionary with board information'''
def process_target_firmware_prototype_file(manufacturer, board, file):
    global verbose, excluded_labels

    if not (file.is_file() and file.name == 'firmware.prototype'):
        return None

    target_name = manufacturer.name + '_' + board.name

    with open(file.path, 'r') as f:
        data = json.load(f)
        print(data)
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
    "build_variants"    = {"VARIANT_1", "VARIANT_2", ... }, (labels)
}
'''
def print_board_information(board_list: list):
    global args
    extra_args = dict()
    json_list = []

    for board in board_list:
        board_variants = [] # Save different board variants (labels)
        board_info = None

        for file in board['files']:
            target = process_target_firmware_prototype_file(board['manufacturer'], board['board'], file)
            if target is None:
                # .px4board files
                if file.name.endswith(".px4board"):
                    label = file.name.split(".px4board")[0]
                    board_variants.append(label)

            if target is not None:
                # firmware.prototype file that defines board information
                board_info = target

        if board_info is None:
            print('Board info is NONE!')
            print(board)

        if board_info is not None:
            board_info['build_variants'] = board_variants
            json_list.append(board_info)

    board_info_dict = { 'board_info' : json_list }

    if args.pretty:
        extra_args['indent'] = 2

    print(json.dumps(board_info_dict, **extra_args))


'''
Go through the board related files and process them depending on the output mode
'''
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

    if enumerate_mode:
        print_board_enumeration(board_list)

    elif boardinfo_mode:
        print_board_information(board_list)


if __name__ == '__main__':
    main()
