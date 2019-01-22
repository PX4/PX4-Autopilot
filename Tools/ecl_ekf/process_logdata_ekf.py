#! /usr/bin/env python

from __future__ import print_function

import argparse
import os
import sys
import csv

from pyulog import *

from analyse_logdata_ekf import analyse_ekf
from detectors import InAirDetector

"""
Performs a health assessment on the ecl EKF navigation estimator data contained in a an ULog file
Outputs a health assessment summary in a csv file named <inputfilename>.mdat.csv
Outputs summary plots in a pdf file named <inputfilename>.pdf
"""

def get_arguments():
    parser = argparse.ArgumentParser(description='Analyse the estimator_status and ekf2_innovation message data')
    parser.add_argument('filename', metavar='file.ulg', help='ULog input file')
    parser.add_argument('--no-plots', action='store_true',
                        help='Whether to only analyse and not plot the summaries for developers.')
    parser.add_argument('--check-level-thresholds', type=str, default=None,
                        help='The csv file of fail and warning test thresholds for analysis.')
    parser.add_argument('--check-description', type=str, default=None,
                        help='The csv file with descriptions of the checks.')
    parser.add_argument('--no-sensor-safety-margin', action='store_true',
                        help='Whether to not cut-off 5s after take-off and 5s before landing '
                             '(for certain sensors that might be influence by proximity to ground).')
    return parser.parse_args()


def is_valid_directory(parser, arg):
    if os.path.isdir(arg):
        # Directory exists so return the directory
        return arg
    else:
        parser.error('The directory {} does not exist'.format(arg))


def process_logdata_ekf(
        filename: str, check_level_dict_filename: str, check_description_filename: str,
        plot: bool = True, sensor_safety_margins: bool = True):

    ## load the log and extract the necessary data for the analyses
    ulog = ULog(filename, None)

    try:
        estimator_status_data = ulog.get_dataset('estimator_status').data
        print('found estimator_status data')
    except:
        print('could not find estimator_status data')
        sys.exit(-1)

    try:
        ekf2_innovations_data = ulog.get_dataset('ekf2_innovations').data
        print('found ekf2_innovation data')
    except:
        print('could not find ekf2_innovation data')
        sys.exit(-1)

    try:
        sensor_preflight_data = ulog.get_dataset('sensor_preflight').data
        print('found sensor_preflight data')
    except:
        print('could not find sensor_preflight data')
        sys.exit(-1)

    try:
        # get the dictionary of fail and warning test thresholds from a csv file
        with open(check_level_dict_filename, 'r') as file:
            reader = csv.DictReader(file)
            check_levels = {row['check_id']: float(row['threshold']) for row in reader}
        print('Using test criteria loaded from {:s}'.format(check_level_dict_filename))
    except:
        print('could not find {:s}'.format(check_level_dict_filename))
        sys.exit(-1)

    try:
        with open(check_description_filename, 'r') as file:
            reader = csv.DictReader(file)
            check_description = {
                row['check_id']: ['NaN', row['check_description']] for row in reader}
        print('Using test description loaded from {:s}'.format(check_description_filename))
    except:
        print('could not find {:s}'.format(check_description_filename))
        sys.exit(-1)

    try:
        in_air = InAirDetector(
            ulog, min_flight_time_seconds=5.0, in_air_margin_seconds=0.0)
        in_air_no_ground_effects = InAirDetector(
            ulog, min_flight_time_seconds=5.0, in_air_margin_seconds=5.0)
    except Exception as e:
        print(str(e))
        sys.exit(-1)

    if in_air_no_ground_effects.take_off is None:
        print('no airtime detected.')
        sys.exit(-1)

    # perform the ekf analysis
    test_results = analyse_ekf(
            estimator_status_data, ekf2_innovations_data, sensor_preflight_data,
        check_levels, check_description, in_air, in_air_no_ground_effects, plot_report=plot,
        output_plot_filename='{:s}.pdf'.format(os.path.splitext(filename)[0]))

    # write metadata to a .csv file
    with open('{:s}.mdat.csv'.format(os.path.splitext(filename)[0]), "w") as file:

        file.write("name,value,description\n")

        # loop through the test results dictionary and write each entry on a separate row, with data comma separated
        # save data in alphabetical order
        key_list = list(test_results.keys())
        key_list.sort()
        for key in key_list:
            file.write(key + "," + str(test_results[key][0]) + "," + test_results[key][1] + "\n")
    print('Test results written to {:s}.mdat.csv'.format(os.path.splitext(filename)))

    if plot:
        print('Plots saved to {:s}.pdf'.format(os.path.splitext(filename)[0]))

    return test_results


def main() -> None:

    args = get_arguments()

    if args.check_level_thresholds is not None:
        check_level_dict_filename = args.check_level_thresholds
    else:
        file_dir = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        check_level_dict_filename = os.path.join(file_dir, "check_level_dict.csv")

    if args.check_description is not None:
        check_description_filename = args.check_description
    else:
        file_dir = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        check_description_filename = os.path.join(file_dir, "check_description.csv")

    test_results = process_logdata_ekf(
        args.filename, check_level_dict_filename, check_description_filename,
        plot=not args.no_plots, sensor_safety_margins=not args.no_sensor_safety_margin)

    # print master test status to console
    if (test_results['master_status'][0] == 'Pass'):
        print('No anomalies detected')
    elif (test_results['master_status'][0] == 'Warning'):
        print('Minor anomalies detected')
    elif (test_results['master_status'][0] == 'Fail'):
        print('Major anomalies detected')
        sys.exit(-1)

if __name__ == '__main__':
    main()
