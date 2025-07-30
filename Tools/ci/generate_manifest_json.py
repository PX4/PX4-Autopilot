#!/usr/bin/env python3

# Usage
# # pretty print all targets
# ./Tools/ci/generate_manifest_json.py | jq
# pretty print filtered targets
# ./Tools/ci/generate_manifest_json.py -f px4_fmu-v6x | jq

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

dconf = Kconfig()

# Supress warning output
dconf.warn_assign_undef = False
dconf.warn_assign_override = False
dconf.warn_assign_redun = False

source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

parser = argparse.ArgumentParser(description='Generate build targets')

parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')
parser.add_argument('-p', '--pretty', dest='pretty', action='store_true',
                    help='Pretty output instead of a single line')
# parser.add_argument('-g', '--groups', dest='group', action='store_true',
#                     help='Groups targets')
# parser.add_argument('-m', '--manifest', dest='manifest', action='store_true',
#                     help='Firmware manifest')
parser.add_argument('-f', '--filter', dest='filter', help='comma separated list of board names to use instead of all')

args = parser.parse_args()
verbose = args.verbose

board_filter = []
if args.filter:
    for board in args.filter.split(','):
        board_filter.append(board)

manifest = []
build_configs = []
grouped_targets = {}
excluded_boards = []
excluded_manufacturers = ['atlflight']
excluded_platforms = ['qurt']
excluded_labels = [
	'stackcheck',
	'nolockstep', 'replay', 'test',
	'uavcanv1', # TODO: fix and enable
]

github_action_config = { 'include': build_configs }
extra_args = {}
if args.pretty:
    extra_args['indent'] = 2

def chunks(arr, size):
    # splits array into parts
    for i in range(0, len(arr), size):
        yield arr[i:i + size]

def comma_targets(targets):
    # turns array of targets into a comma split string
    return ",".join(targets)

def process_variants(targets):
    # returns the
    return [{"name": v} for v in targets]

def process_target(px4board_file, target_name, defconfig_data, protoconfig_data):
    # reads through the board file and grabs
    # useful information for building
    hardware = {
        'architecture': '',
        'vendor_id': '',
        'product_id': '',
        'chip': '',
        'productstr': '',
    }
    manifest_item = {
        'name': target_name,
        'description': '',
        'manufacturer': '',
        'hardware': {},
        'toolchain': '',
        'artifact': '',
        'sha256sum': '',
        'summary': '',
        'image_maxsize': '',
        'board_id': '',
    }

    board_data = process_boardfile(px4board_file)
    manifest_item['toolchain'] = board_data.get('toolchain', '')
    manifest_item['manufacturer'] = board_data.get('manufacturer', '')

    # prototype is required per-board
    manifest_item['description'] = protoconfig_data.get('description', '')
    manifest_item['board_id'] = protoconfig_data.get('board_id', '')
    manifest_item['summary'] = protoconfig_data.get('summary', '')
    manifest_item['image_maxsize'] = protoconfig_data.get('image_maxsize', '')

    if defconfig_data:
        if defconfig_data.get('manufacturer'):
            manifest_item['manufacturer'] = defconfig_data['manufacturer']
        hw = defconfig_data.get('hardware', {})
        for k in ('vendor_id', 'product_id', 'chip', 'productstr', 'architecture'):
            if hw.get(k):
                hardware[k] = hw[k]

    manifest_item['hardware'] = hardware
    return manifest_item

def process_boardfile(boardtarget_file):
    return_boardfile = {
        'toolchain': '',
        'boardfile': '',
        'manufacturer': '',
        'architecture': '',
    }
    if boardtarget_file.endswith("default.px4board") or \
        boardtarget_file.endswith("performance-test.px4board") or \
        boardtarget_file.endswith("bootloader.px4board"):
        kconf.load_config(boardtarget_file, replace=True)
    else: # Merge config with default.px4board
        default_kconfig = re.sub(r'[a-zA-Z\d_-]+\.px4board', 'default.px4board', boardtarget_file)
        kconf.load_config(default_kconfig, replace=True)
        kconf.load_config(boardtarget_file, replace=False)

    return_boardfile['boardfile'] = os.path.abspath(boardtarget_file)

    if "BOARD_TOOLCHAIN" in kconf.syms:
        return_boardfile['toolchain'] = kconf.syms["BOARD_TOOLCHAIN"].str_value

    if "CONFIG_CDCACM_VENDORSTR" in kconf.syms:
        return_boardfile['manufacturer'] = kconf.syms["CONFIG_CDCACM_VENDORSTR"].str_value

    if "BOARD_ARCHITECTURE" in kconf.syms:
        return_boardfile['architecture'] = kconf.syms["BOARD_ARCHITECTURE"].str_value

    return return_boardfile

def load_defconfig(defconfig_file):
    if not os.path.isfile(defconfig_file):
        return None

    hardware = {
        'architecture': '',
        'vendor_id': '',
        'product_id': '',
        'chip': '',
        'productstr': '',
    }
    manifest_item = {
        'manufacturer': '',
        'hardware': hardware,
    }
    defconfig = {}
    with open(defconfig_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, value = line.split('=', 1)
            value = value.strip('"')
            defconfig[key] = value

    manifest_item['manufacturer'] = defconfig.get('CONFIG_CDCACM_VENDORSTR', '')
    hardware['vendor_id'] = defconfig.get('CONFIG_CDCACM_VENDORID', '')
    hardware['product_id'] = defconfig.get('CONFIG_CDCACM_PRODUCTID', '')
    hardware['chip'] = defconfig.get('CONFIG_ARCH_CHIP', '')
    hardware['productstr'] = defconfig.get('CONFIG_CDCACM_PRODUCTSTR', '')
    hardware['architecture'] = defconfig.get('CONFIG_ARCH', '')

    return manifest_item

def process_proto(protoconfig_file):
    return_proto = {
        'board_id': '',
        'description': '',
        'summary': '',
        'image_maxsize': ''
    }
    proto = {}
    with open(protoconfig_file, "r") as f:
        proto = json.load(f)

    return_proto['board_id'] = proto['board_id']
    return_proto['description'] = proto['description']
    return_proto['summary'] = proto['summary']
    return_proto['image_maxsize'] = proto['image_maxsize']

    return return_proto

# Look for board targets in the ./boards directory
if(verbose):
    print("=======================")
    print("= scanning for boards =")
    print("=======================")

targets_list = {}

# assume this file lives under Tools/ci/
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
boards_root = os.path.join(repo_root, 'boards')

for manufacturer in os.scandir(boards_root):
    if not manufacturer.is_dir():
        continue
    if manufacturer.name in excluded_manufacturers:
        if verbose: print(f'excluding manufacturer {manufacturer.name}')
        continue

    for board in os.scandir(manufacturer.path):
        if not board.is_dir():
            continue

        # resolve per-board shared files up front
        proto_path = os.path.join(board.path, 'firmware.prototype')
        protoconfig_data = None
        if os.path.isfile(proto_path):
            try:
                protoconfig_data =  process_proto(proto_path)
            except Exception as e:
                if verbose: print(f'warning: failed to parse {proto_path}: {e}')

        defconfig_file = os.path.join(board.path, 'nuttx-config', 'nsh', 'defconfig')
        defconfig_data = load_defconfig(defconfig_file) if os.path.isfile(defconfig_file) else None

        for f in os.scandir(board.path):
            if not f.is_file():
                continue

            if f.name.endswith('.px4board'):
                board_name = manufacturer.name + '_' + board.name
                label = f.name[:-9]
                target_name = manufacturer.name + '_' + board.name + '_' + label

                if board_filter and not board_name in board_filter:
                    if verbose: print(f'excluding board {board_name} ({target_name})')
                    continue
                if board_name in excluded_boards:
                    if verbose: print(f'excluding board {board_name} ({target_name})')
                    continue
                if label in excluded_labels:
                    if verbose: print(f'excluding label {label} ({target_name})')
                    continue

                if protoconfig_data is None:
                    if verbose: print(f'skipping {target_name}: missing firmware.prototype')
                    continue

                target = process_target(f.path,
                                        target_name,
                                        defconfig_data,
                                        protoconfig_data)

                if target is not None:
                    build_configs.append(target)

print(json.dumps(build_configs, **extra_args))
