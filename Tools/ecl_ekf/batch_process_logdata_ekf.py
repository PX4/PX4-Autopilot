#! /usr/bin/env python3
"""
Runs process_logdata_ekf.py on the .ulg files in the supplied directory. ulog files are skipped from the analysis, if a
 corresponding .pdf file already exists (unless the overwrite flag was set).
"""
# -*- coding: utf-8 -*-

import argparse
import os, glob

from process_logdata_ekf import process_logdata_ekf

def get_arguments():
    parser = argparse.ArgumentParser(description='Analyse the estimator_status and ekf2_innovation message data for the'
                                                 ' .ulg files in the specified directory')
    parser.add_argument("directory_path")
    parser.add_argument('-o', '--overwrite', action='store_true',
                        help='Whether to overwrite an already analysed file. If a file with .pdf extension exists for a .ulg'
                             'file, the log file will be skipped from analysis unless this flag has been set.')
    parser.add_argument('--no-plots', action='store_true',
                        help='Whether to only analyse and not plot the summaries for developers.')
    parser.add_argument('--check-level-thresholds', type=str, default=None,
                        help='The csv file of fail and warning test thresholds for analysis.')
    parser.add_argument('--check-table', type=str, default=None,
                        help='The csv file with descriptions of the checks.')
    parser.add_argument('--no-sensor-safety-margin', action='store_true',
                        help='Whether to not cut-off 5s after take-off and 5s before landing '
                             '(for certain sensors that might be influence by proximity to ground).')
    return parser.parse_args()


def main() -> None:

    args = get_arguments()

    if args.check_level_thresholds is not None:
        check_level_dict_filename = args.check_level_thresholds
    else:
        file_dir = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        check_level_dict_filename = os.path.join(file_dir, "check_level_dict.csv")

    if args.check_table is not None:
        check_table_filename = args.check_table
    else:
        file_dir = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        check_table_filename = os.path.join(file_dir, "check_table.csv")

    ulog_directory = args.directory_path

    # get all the ulog files found in the specified directory and in subdirectories
    ulog_files = glob.glob(os.path.join(ulog_directory, '**/*.ulg'), recursive=True)
    print("found {:d} .ulg files in {:s}".format(len(ulog_files), ulog_directory))

    # remove the files already analysed unless the overwrite flag was specified. A ulog file is consired to be analysed if
    # a corresponding .pdf file exists.'
    if not args.overwrite:
        print("skipping already analysed ulg files.")
        ulog_files = [ulog_file for ulog_file in ulog_files if
                      not os.path.exists('{}.pdf'.format(ulog_file))]

    n_files = len(ulog_files)

    print("analysing the {:d} .ulg files".format(n_files))

    i = 1
    n_skipped = 0
    # analyse all ulog files
    for ulog_file in ulog_files:
        print('analysing file {:d}/{:d}: {:s}'.format(i, n_files, ulog_file))

        try:
            _ = process_logdata_ekf(
                ulog_file, check_level_dict_filename, check_table_filename,
                plot=not args.no_plots, sensor_safety_margins=not args.no_sensor_safety_margin)

        except Exception as e:
            print(str(e))
            print('an exception occurred, skipping file {:s}'.format(ulog_file))
            n_skipped = n_skipped + 1

        i = i + 1

    print('{:d}/{:d} files analysed, {:d} skipped.'.format(n_files-n_skipped, n_files, n_skipped))


if __name__ == '__main__':
    main()
