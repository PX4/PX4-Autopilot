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
parser.add_argument('-g', '--groups', dest='group', action='store_true',
                    help='Groups targets')

args = parser.parse_args()
verbose = args.verbose

build_configs = []
grouped_targets = {}
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
    group = None

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
        if platform == 'posix':
            container = 'px4io/px4-dev-base-focal:2021-09-08'
            group = 'base'
            if toolchain:
                if toolchain.startswith('aarch64'):
                    container = 'px4io/px4-dev-aarch64:2022-08-12'
                    group = 'aarch64'
                elif toolchain == 'arm-linux-gnueabihf':
                    container = 'px4io/px4-dev-armhf:2023-06-26'
                    group = 'armhf'
                else:
                    if verbose: print(f'unmatched toolchain: {toolchain}')
        elif platform == 'nuttx':
            container = 'px4io/px4-dev-nuttx-focal:2022-08-12'
            group = 'nuttx'
        else:
            if verbose: print(f'unmatched platform: {platform}')

        ret = {'target': target_name, 'container': container}
        if(args.group):
            ret['arch'] = group

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
                # print(target)
                if (args.group and target is not None):
                    if (target['arch'] not in grouped_targets):
                        grouped_targets[target['arch']] = {}
                        grouped_targets[target['arch']]['container'] = target['container']
                        grouped_targets[target['arch']]['manufacturers'] = {}
                    if(manufacturer.name not in grouped_targets[target['arch']]['manufacturers']):
                        grouped_targets[target['arch']]['manufacturers'][manufacturer.name] = {}
                        grouped_targets[target['arch']]['manufacturers'][manufacturer.name] = []
                    grouped_targets[target['arch']]['manufacturers'][manufacturer.name].append(target_name)
                if target is not None:
                    build_configs.append(target)

def comma_targets(targets):
    return ",".join(targets)

github_action_config = { 'include': build_configs }
extra_args = {}
if args.pretty:
    extra_args['indent'] = 2

if (args.group):
    final_groups = []
    temp_group = []
    group_number = {}
    last_man = ''
    last_arch = ''
    for arch in grouped_targets:
        if(last_arch == ''):
            last_arch = arch
        if(arch not in group_number):
                group_number[arch] = 0

        if(last_arch != arch and len(temp_group) > 0):

            group_name = arch + "-" + str(group_number[arch])
            final_groups.append({
                "container": grouped_targets[arch]['container'],
                "targets": comma_targets(temp_group),
                "arch": arch,
                "group": group_name
            })
            last_arch = arch
            group_number[arch] += 1
            temp_group = []
        for man in grouped_targets[arch]['manufacturers']:
            for tar in grouped_targets[arch]['manufacturers'][man]:
                if(last_man != man):
                    if(len(grouped_targets[arch]['manufacturers'][man]) > 10):
                        group_name = arch + "-" + man
                        last_man = man
                        final_groups.append({
                            "container": grouped_targets[arch]['container'],
                            "targets": comma_targets(grouped_targets[arch]['manufacturers'][man]),
                            "arch": arch,
                            "group": group_name
                        })
                    else:
                        temp_group.append(tar)

            if(len(temp_group) > 15):
                group_name = arch + "-" + str(group_number[arch])
                final_groups.append({
                    "container": grouped_targets[arch]['container'],
                    "targets": comma_targets(temp_group),
                    "arch": arch,
                    "group": group_name
                })
                last_arch = arch
                group_number[arch] += 1
                temp_group = []
    print(json.dumps({ "include": final_groups }, **extra_args))
else:
    print(json.dumps(github_action_config, **extra_args))
