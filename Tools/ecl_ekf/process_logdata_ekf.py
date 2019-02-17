#! /usr/bin/env python

from __future__ import print_function

import argparse
import os, sys

from pyulog import *

from analyse_logdata_ekf import analyse_ekf

"""
Performs a health assessment on the ecl EKF navigation estimator data contained in a an ULog file
Outputs a health assessment summary in a csv file named <inputfilename>.mdat.csv
Outputs summary plots in a pdf file named <inputfilename>.pdf
"""

parser = argparse.ArgumentParser(description='Analyse the estimator_status and ekf2_innovation message data')
parser.add_argument('filename', metavar='file.ulg', help='ULog input file')
parser.add_argument('--no-plots', action='store_true',
                    help='Whether to only analyse and not plot the summaries for developers.')
parser.add_argument('--check-level-thresholds', type=str, default=None,
                    help='The csv file of fail and warning test thresholds for analysis.')
parser.add_argument('--no-sensor-safety-margin', action='store_true',
                    help='Whether to not cut-off 5s after take-off and 5s before landing '
                         '(for certain sensors that might be influence by proximity to ground).')

def is_valid_directory(parser, arg):
    if os.path.isdir(arg):
        # Directory exists so return the directory
        return arg
    else:
        parser.error('The directory {} does not exist'.format(arg))

args = parser.parse_args()

## load the log and extract the necessary data for the analyses
ulog = ULog(args.filename, None)
data = ulog.data_list

# extract data from EKF status message
estimator_status_data = {}
try:
    estimator_status_data = ulog.get_dataset('estimator_status').data;
except (KeyError, IndexError, ValueError) as error:
    print(type(error), "(estimator_status):", error)

# extract data from EKF innovations message
ekf2_innovations_data = {}
try:
    ekf2_innovations_data = ulog.get_dataset('ekf2_innovations').data;
except (KeyError, IndexError, ValueError) as error:
    print(type(error), "(ekf2_innovations):", error)

# extract data from sensor preflight check message
sensor_preflight_data = {}
try:
    sensor_preflight_data = ulog.get_dataset('sensor_preflight').data;
except (KeyError, IndexError, ValueError) as error:
    print(type(error), "(sensor_preflight):", error)

if args.check_level_thresholds:
    check_level_dict_filename = args.check_level_thresholds
else:
    file_dir = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
    check_level_dict_filename = os.path.join(file_dir, "check_level_dict.csv")

# get the dictionary of fail and warning test thresholds from a csv file
with open(check_level_dict_filename, 'r') as file:
    check_levels = {}
    for line in file:
        x = line.split(",")
        a = x[0]
        b = x[1]
        check_levels[a] = float(b)

print('Using test criteria loaded from {:s}'.format(check_level_dict_filename))

# perform the ekf analysis
test_results = analyse_ekf(
    estimator_status_data, ekf2_innovations_data, sensor_preflight_data,
    check_levels, plot=not args.no_plots, output_plot_filename=args.filename + ".pdf",
    late_start_early_ending=not args.no_sensor_safety_margin)

# write metadata to a .csv file
with open(args.filename + ".mdat.csv", "w") as file:

    file.write("name,value,description\n")

    # loop through the test results dictionary and write each entry on a separate row, with data comma separated
    # save data in alphabetical order
    key_list = list(test_results.keys())
    key_list.sort()
    for key in key_list:
        file.write(key+","+str(test_results[key][0])+","+test_results[key][1]+"\n")

print('Test results written to {:s}.mdat.csv'.format(args.filename))

if not args.no_plots:
    print('Plots saved to {:s}.pdf'.format(args.filename))

# print master test status to console
if (test_results['master_status'][0] == 'Pass'):
    print('No anomalies detected')
elif (test_results['master_status'][0] == 'Warning'):
    print('Minor anomalies detected')
elif (test_results['master_status'][0] == 'Fail'):
    print('Major anomalies detected')
    sys.exit(-1)
