#!/usr/bin/env python3
""" Script to generate a JSON config with all build targets (for CI) """

import argparse
import os
import sys
import json
import re
from kconfiglib import Kconfig

kconf = Kconfig()

# Supress warning output
kconf.warn_assign_undef = False
kconf.warn_assign_override = False
kconf.warn_assign_redun = False

source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

parser = argparse.ArgumentParser(description='Generate build targets')

parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')
parser.add_argument('-p', '--pretty', dest='pretty', action='store_true',
                    help='Pretty output instead of a single line')

args = parser.parse_args()
verbose = args.verbose

build_configs = []
excluded_boards = ['modalai_voxl2', 'px4_ros2']  # TODO: fix and enable
excluded_manufacturers = ['atlflight']
excluded_platforms = ['qurt']
excluded_labels = [
    'stackcheck',
    'nolockstep', 'replay', 'test',
    'uavcanv1', # TODO: fix and enable
    ]

def process_target(px4board_file, target_name):
    ret = None
    platform = None
    toolchain = None

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

for manufacturer in os.scandir(os.path.join(source_dir, 'boards')):
    if not manufacturer.is_dir():
        continue
    if manufacturer.name in excluded_manufacturers:
        if verbose: print(f'excluding manufacturer {manufacturer.name}')
        continue

    for board in os.scandir(manufacturer.path):
        if not board.is_dir():
            continue

        for files in os.scandir(board.path):
            if files.is_file() and files.name.endswith('.px4board'):

                board_name = manufacturer.name + '_' + board.name
                label = files.name[:-9]
                target_name = manufacturer.name + '_' + board.name + '_' + label

                if board_name in excluded_boards:
                    if verbose: print(f'excluding board {board_name} ({target_name})')
                    continue

                if label in excluded_labels:
                    if verbose: print(f'excluding label {label} ({target_name})')
                    continue
                target = process_target(files.path, target_name)
                if target is not None:
                    build_configs.append(target)


github_action_config = { 'include': build_configs }
extra_args = {}
if args.pretty:
    extra_args['indent'] = 2
print(json.dumps(github_action_config, **extra_args))

