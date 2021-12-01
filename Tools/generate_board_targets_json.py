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
parser.add_argument('-a', '--all_variants', dest='all_variants', action='store_true',
                    help='Prints targets into a single all_variants_* target')
parser.add_argument('-b', '--bloaty', dest='bloaty', action='store_true',
                    help='Includes all bloaty targets')

args = parser.parse_args()
verbose = args.verbose

build_configs = []
excluded_manufacturers = ['atlflight']
excluded_platforms = ['qurt']
excluded_labels = [
    'stackcheck',
    'nolockstep', 'replay', 'test',
    'uavcanv1' # TODO: fix and enable
    ]
bloaty_helpers = [
    'bloaty_compileunits',
    'bloaty_inlines',
    'bloaty_segments',
    'bloaty_symbols',
    'bloaty_templates',
    'bloaty_ram',
    'bloaty_compare_master',
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

def process_bloaty(target_path, target_name):
    response = []
    for bloat in bloaty_helpers:
        bloaty_name = target_name + ' ' + bloat + ' || true'
        processed_target = process_target(target_path, bloaty_name)
        response.append(processed_target)
    return response

# Look up boards from each manufacturer
for manufacturer in os.scandir(os.path.join(source_dir, 'boards')):
    if not manufacturer.is_dir():
        continue
    if manufacturer.name in excluded_manufacturers:
        if verbose: print(f'excluding manufacturer {manufacturer.name}')
        continue

    for board in os.scandir(manufacturer.path):
        # Only boards are directories don't proceed if otherwise
        if not board.is_dir():
            continue

        if args.all_variants:
            # The all_variants target makes all targets for a board
            target_name = 'all_variants_' + manufacturer.name + '_' + board.name
            default_target_path = f'{board.path}/default.px4board'
            target = process_target(default_target_path, target_name)
            if target is not None:
                build_configs.append(target)
            if args.bloaty and target is not None:
                # bloaty targets
                bloat_target_name = manufacturer.name + '_' + board.name
                bloaty_targets = process_bloaty(default_target_path, bloat_target_name)
                build_configs += bloaty_targets
        else:
            # Each board can have multiple variant targets
            for files in os.scandir(board.path):
                if files.is_file() and files.name.endswith('.px4board'):
                    label = files.name[:-9]
                    target_name = manufacturer.name + '_' + board.name + '_' + label
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

