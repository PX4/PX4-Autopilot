#!/usr/bin/env python3
"""
Script to print out the board information manifest.

This includes generic information about all the board targets, and is intended to be used in
QGC / Ground Control Station side to have an up-to-date information on which board targets are supported,
where the build binaries can be downloaded from, and so on.

The format in JSON will be something like this:
{
    "board_info":
    [
        {
            "board_name"        = "HUMAN_READABLE_BOARD_NAME",      (summary)
            "target_name"       = "TARGET_NAME_FOR_BUILD",          (manufacturer_board_label)
            "description"       = "DESCRIPTION",                    (description)
            "board_id"          = "BOOTLOADER_ID",
            "image_maxsize"     = "FLASH_SIZE",
            "build_variants"    = [ {"name": "VARIANT_1"}, ... ]    (labels)
        },
        ...
    ],

    "binary_urls":
    {
        "stable":
        {
            "url": "http://px4-travis.s3.amazonaws.com/Firmware/stable/${target_name}_${build_variant}.px4"
        },
        "beta":
        {
            "url": "http://px4-travis.s3.amazonaws.com/Firmware/beta/${target_name}_${build_variant}.px4"
        },
        ...
    }
}
"""

import argparse
import os
import json
from kconfiglib import Kconfig

parser = argparse.ArgumentParser(description='Generate build targets')

parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')
parser.add_argument('-p', '--pretty', dest='pretty', action='store_true',
                    help='Pretty output instead of a single line')
args = parser.parse_args()

## GLOBAL VARIABLES
verbose = args.verbose
build_configs = []
excluded_manufacturers = ['atlflight']
excluded_labels = [
    'stackcheck', 'nolockstep', 'replay', 'test',
    'uavcanv1' # TODO: fix and enable
    ]

DEFCONFIG_FILE_PATH = 'nuttx-config/nsh/defconfig'
DEFCONFIG_PRODUCTID_KEY = 'CDCACM_PRODUCTID'
DEFCONFIG_PRODUCTSTR_KEY = 'CDCACM_PRODUCTSTR' # Property that defines how the board shows up as when connected via USB
DEFCONFIG_VENDORID_KEY = 'CDCACM_VENDORID'
DEFCONFIG_VENDORSTR_KEY = 'CDCACM_VENDORSTR'

FIRMWRAE_PROTOTYPE_FILE_PATH = 'firmware.prototype'

# Create a Kconfig object with the usbdev driver's Kconfig configuration, which includes definition
# for the CDCACM_* configuration parameters that we will be using.
# Also supress warning output while parsing KConfig
kconf = Kconfig(filename = 'platforms/nuttx/NuttX/nuttx/drivers/usbdev/Kconfig', warn=False)

''' Processes firmware.prototype file in target directory '''
def process_target_firmware_prototype_file(manufacturer, board, file_path):
    if not (os.path.isfile(file_path) and os.path.basename(file_path) == 'firmware.prototype'):
        if verbose: print("Can't process the firmware.prototype file because it isn't!! : {}".format(file_path))
        return None
    target_name = manufacturer + '_' + board
    with open(file_path, 'r') as f:
        data = json.load(f)
        board_info_dict = dict()
        board_info_dict['board_name'] = data['summary']
        board_info_dict['target_name'] = target_name
        board_info_dict['description'] = data['description']
        board_info_dict['board_id'] = data['board_id']
        board_info_dict['image_max_size'] = data['image_maxsize']
        board_info_dict['build_variants'] = []
        return board_info_dict

''' Processes defconfig file in target directory '''
def process_target_defconfig_file(file_path):
    if not (os.path.isfile(file_path) and os.path.basename(file_path) == 'defconfig'):
        if verbose: print("Can't process the defconfig file because it isn't!! : {}".format(file_path))
        return None
    board_info_dict = dict()
    try:
        kconf.load_config(file_path, replace=True) # Load the Kconfig symbols from the file
        if DEFCONFIG_PRODUCTID_KEY in kconf.syms:
            board_info_dict['product_id'] = kconf.syms[DEFCONFIG_PRODUCTID_KEY].str_value
        if DEFCONFIG_VENDORID_KEY in kconf.syms:
            board_info_dict['vendor_id'] = kconf.syms[DEFCONFIG_VENDORID_KEY].str_value
        if DEFCONFIG_PRODUCTSTR_KEY in kconf.syms:
            board_info_dict['product_name'] = kconf.syms[DEFCONFIG_PRODUCTSTR_KEY].str_value
        if DEFCONFIG_VENDORSTR_KEY in kconf.syms:
            board_info_dict['vendor_name'] = kconf.syms[DEFCONFIG_VENDORSTR_KEY].str_value
    except:
        return None

    # Sanity check to verify we have a properly parsed defconfig file
    # This can happen for some targets where the defconfig file doesn't include key strings like vendor-id!
    if board_info_dict['product_id'] == '':
        return None

    return board_info_dict

''' Function that receives board list and goes through the files to create the final Board Information metadata JSON '''
def print_board_information(board_list: list):
    global args, verbose, excluded_labels
    # Objects that we will be using to create the JSON at the end of this function
    board_info_list = []

    for board in board_list:
        board_info = dict()

        # Process the defconfig file to get USB Autoconnect info
        if (os.path.isfile(os.path.join(board['path'], DEFCONFIG_FILE_PATH))):
            defconfig_result = process_target_defconfig_file(os.path.join(board['path'], DEFCONFIG_FILE_PATH))
            if defconfig_result is not None:
                board_info = {**board_info, **defconfig_result}
            else:
                if verbose: print("Error, Couldn't parse defconfig for the board: {} - {}".format(board['manufacturer'], board['board']))
        else:
            if verbose: print("Error, didn't detect valid defconfig for the board! : {} - {}".format(board['manufacturer'], board['board']))

        # Process the firmware.prototype file to get general information
        if (os.path.isfile(os.path.join(board['path'], FIRMWRAE_PROTOTYPE_FILE_PATH))):
            firmware_prototype_result = process_target_firmware_prototype_file(board['manufacturer'], board['board'], os.path.join(board['path'], FIRMWRAE_PROTOTYPE_FILE_PATH))
            if firmware_prototype_result is not None:
                board_info = {**board_info, **firmware_prototype_result}
            else:
                if verbose: print("Firmware.prototype result was None for : {} - {}".format(board['manufacturer'], board['board']))
        else:
            if verbose: print("Error, didn't detect valid firmware.prototype for the board! : {} - {}".format(board['manufacturer'], board['board']))

        # Process the *.px4board files for getting build variants
        board_variants = []
        for file in os.scandir(board['path']):
            if (os.path.isfile(file) and file.name.endswith(".px4board")):
                label = file.name.split(".px4board")[0]
                if label not in excluded_labels:
                    board_variants.append({"name": label}) # For now we only have the 'name' attribute (e.g. default, rtps, etc.)
        board_info['build_variants'] = board_variants

        board_info_list.append(board_info)

    # Include binary URL information
    binary_urls_dict = {"stable" : {"url" : "http://px4-travis.s3.amazonaws.com/Firmware/stable/${target_name}_${build_variant}.px4"},
                        "beta" : {"url" : "http://px4-travis.s3.amazonaws.com/Firmware/beta/${target_name}_${build_variant}.px4"},
                        "main" : {"url" : "http://px4-travis.s3.amazonaws.com/Firmware/main/${target_name}_${build_variant}.px4"},
                        }

    # Final dictionary that will be turned into the JSON file
    final_json_dict = { 'board_info' : board_info_list,
                        'binary_urls' : binary_urls_dict}

    extra_args = dict()         # Extra arg that will be added to the JSON
    if args.pretty:
        extra_args['indent'] = 2

    print(json.dumps(final_json_dict, **extra_args))

''' Go through the boards/* to get list of boards and it's directory '''
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
            board_list.append({'manufacturer': manufacturer.name, 'board' : board.name, 'path' : board.path})

    # Process the board list and output the JSON
    print_board_information(board_list)

if __name__ == '__main__':
    main()
