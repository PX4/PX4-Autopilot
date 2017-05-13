#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import os

"""
Runs process_logdata_ekf.py on all the files in the suplied directory with a .ulg extension
"""

parser = argparse.ArgumentParser(description='Analyse the estimator_status and ekf2_innovation message data for all .ulg files in the specified directory')
parser.add_argument("directory_path")

def is_valid_directory(parser, arg):
    if os.path.isdir(arg):
        # Directory exists so return the directory
        return arg
    else:
        parser.error('The directory {} does not exist'.format(arg))

args = parser.parse_args()
ulog_directory = args.directory_path
print("\n"+"analysing all .ulog files in "+ulog_directory)
# Run the analysis script on all the log files found in the specified directory
for file in os.listdir(ulog_directory):
    if file.endswith(".ulg"):
        print("\n"+"loading "+file+" for analysis")
        os.system("python process_logdata_ekf.py "+ulog_directory+"/"+file)
