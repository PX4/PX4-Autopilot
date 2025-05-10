#!/usr/bin/env python3

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

def process_target(px4board_file, target_name, defconfig_data, defconfig_file):
    # reads through the board file and grabs
    # useful information for building
    ret = None
    platform = None
    toolchain = None
    group = None
    hardware = {
        'architecture': '',
        'vendor_id': '',
        'product_id': '',
        'chip': '',
        'productstr': '',
    }
    manifest_item = {
        'name': target_name,
        'manufacturer': '',
        'hardware': {},
        'toolchain': '',
        'artifact': '',
        'sha256sum': ''
    }

    board_data = process_boardfile(px4board_file)
    # manifest_item['boardfile'] = board_data['boardfile']
    # manifest_item['debug'] = os.path.abspath(defconfig_file)
    manifest_item['toolchain'] = board_data['toolchain']
    manifest_item['manufacturer'] = board_data['manufacturer']

    if defconfig_data is not None:
        manifest_item['manufacturer'] = defconfig_data['manufacturer']
        hardware = defconfig_data['hardware']
        hardware['architecture'] = board_data['architecture']

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
    hardware = {
        'architecture': '',
        'vendor_id': '',
        'product_id': '',
        'chip': '',
        'description': '',
    }
    manifest_item = {
        'name': '',
        'manufacturer': '',
        'hardware': {},
        'toolchain': '',
        'artifact': '',
        'sha256sum': '',
        'boardfile': '',
        'debug': ''
    }
    defconfig_data = process_defconfig(defconfig_file)
    manifest_item['manufacturer'] = defconfig_data['manufacturer']
    # hardware['architecture'] = defconfig_data['architecture']
    hardware['vendor_id'] = defconfig_data['vendor_id']
    hardware['product_id'] = defconfig_data['product_id']
    hardware['chip'] = defconfig_data['chip']
    hardware['description'] = defconfig_data['description']
    manifest_item['hardware'] = hardware

    return manifest_item

def process_defconfig(defconfig_file):
    return_defconfig = {
        'manufacturer': '',
        'vendor_id': '',
        'product_id': '',
        'chip': '',
        'description': '',
    }
    defconfig = {}

    defconfig_available = False
    abs_defconfig_path = os.path.abspath(defconfig_file)
    if(os.path.isfile(defconfig_file)):
       defconfig_available = True

    if(defconfig_available):
        with open(defconfig_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                value = value.strip('"')
                defconfig[key] = value

        if "CONFIG_CDCACM_VENDORSTR" in defconfig.keys():
            return_defconfig['manufacturer'] = defconfig.get("CONFIG_CDCACM_VENDORSTR")

        if "CONFIG_CDCACM_VENDORID" in defconfig.keys():
            return_defconfig['vendor_id'] = defconfig.get("CONFIG_CDCACM_VENDORID")

        if "CONFIG_CDCACM_PRODUCTID" in defconfig.keys():
            return_defconfig['product_id'] = defconfig.get("CONFIG_CDCACM_PRODUCTID")

        if "CONFIG_ARCH_CHIP" in defconfig.keys():
            return_defconfig['chip'] = defconfig.get("CONFIG_ARCH_CHIP")

        if "CONFIG_CDCACM_PRODUCTSTR" in defconfig.keys():
            return_defconfig['description'] = defconfig.get("CONFIG_CDCACM_PRODUCTSTR")

    return return_defconfig

# Look for board targets in the ./boards directory
if(verbose):
    print("=======================")
    print("= scanning for boards =")
    print("=======================")

targets_list = {}
# list of build targets
# [
# 	{
# 		"name": "px4_fmu-v6x",
# 		"manufacturer": "CONFIG_CDCACM_VENDORSTR",
# 		"hardware": {
# 			"architecture": "CONFIG_BOARD_ARCHITECTURE",
# 			"vendor_id": "CONFIG_CDCACM_VENDORID",
# 			"product_id": "CONFIG_CDCACM_PRODUCTID",
# 			"chip": "CONFIG_ARCH_CHIP",
# 			"description": "CONFIG_CDCACM_PRODUCTSTR",
# 		},
# 		"toolchain": "",
# 		"artifact": "",
# 		"sha256sum": ""
# 	    }
# ]



for manufacturer in os.scandir(os.path.join(source_dir, '../boards')):
    if not manufacturer.is_dir():
        continue
    if manufacturer.name in excluded_manufacturers:
        if verbose: print(f'excluding manufacturer {manufacturer.name}')
        continue

    for board in os.scandir(manufacturer.path):
        if not board.is_dir():
            continue

        defconfig_file = os.path.join(board.path, 'nuttx-config/nsh/defconfig')
        defconfig_data = load_defconfig(defconfig_file)

        for files in os.scandir(board.path):

            if files.is_file() and files.name.endswith('.px4board'):

                board_name = manufacturer.name + '_' + board.name
                label = files.name[:-9]
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

                target = process_target(files.path, target_name, defconfig_data, defconfig_file)

                if target is not None:
                    build_configs.append(target)

print(json.dumps(build_configs, **extra_args))
