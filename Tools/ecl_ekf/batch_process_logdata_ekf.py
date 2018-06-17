#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import os, glob

"""
Runs process_logdata_ekf.py on the .ulg files in the supplied directory. ulog files are skipped from the analysis, if a 
 corresponding .pdf file already exists (unless the overwrite flag was set). 
"""

parser = argparse.ArgumentParser(description='Analyse the estimator_status and ekf2_innovation message data for the'
                                             ' .ulg files in the specified directory')
parser.add_argument("directory_path")
parser.add_argument('-o', '--overwrite', action='store_true',
                    help='Whether to overwrite an already analysed file. If a file with .pdf extension exists for a .ulg'
                         'file, the log file will be skipped from analysis unless this flag has been set.')

def is_valid_directory(parser, arg):
    if os.path.isdir(arg):
        # Directory exists so return the directory
        return arg
    else:
        parser.error('The directory {} does not exist'.format(arg))

args = parser.parse_args()
ulog_directory = args.directory_path
print("\n"+"analysing the .ulg files in "+ulog_directory)

# get all the ulog files found in the specified directory
ulog_files = glob.glob(os.path.join(ulog_directory, '*.ulg'))

# remove the files already analysed unless the overwrite flag was specified. A ulog file is consired to be analysed if
# a corresponding .pdf file exists.'
if not args.overwrite:
    print("skipping already analysed ulg files.")
    ulog_files = [ulog_file for ulog_file in ulog_files if not os.path.exists('{}.pdf'.format(ulog_file))]

# analyse all ulog files
for ulog_file in ulog_files:
    print("\n"+"loading "+ulog_file +" for analysis")
    os.system("python process_logdata_ekf.py '{}'".format(ulog_file))
