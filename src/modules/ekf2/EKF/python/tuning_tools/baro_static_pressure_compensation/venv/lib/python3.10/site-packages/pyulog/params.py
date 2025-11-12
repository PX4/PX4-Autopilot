#! /usr/bin/env python
"""
Extract parameters from an ULog file
"""

from __future__ import print_function

import argparse
import sys

from .core import ULog
#pylint: disable=unused-variable, too-many-branches

def get_defaults(ulog, default):
    """ get default params from ulog """
    assert ulog.has_default_parameters, "Log does not contain default parameters"

    if default == 'system': return ulog.get_default_parameters(0)
    if default == 'current_setup': return ulog.get_default_parameters(1)
    raise Exception('invalid value \'{}\' for --default'.format(default))

def main():
    """Commande line interface"""
    parser = argparse.ArgumentParser(description='Extract parameters from an ULog file')
    parser.add_argument('filename', metavar='file.ulg', help='ULog input file')

    parser.add_argument('-l', '--delimiter', dest='delimiter', action='store',
                        help='Use delimiter in CSV (default is \',\')', default=',')

    parser.add_argument('-i', '--initial', dest='initial', action='store_true',
                        help='Only extract initial parameters. (octave|csv)', default=False)

    parser.add_argument('-t', '--timestamps', dest='timestamps', action='store_true',
                        help='Extract changed parameters with timestamps. (csv)', default=False)

    parser.add_argument('-f', '--format', dest='format', action='store', type=str,
                        help='csv|octave|qgc', default='csv')

    parser.add_argument('output_filename', metavar='params.txt',
                        type=argparse.FileType('w'), nargs='?',
                        help='Output filename (default=stdout)', default=sys.stdout)

    parser.add_argument('--ignore', dest='ignore', action='store_true',
                        help='Ignore string parsing exceptions', default=False)

    parser.add_argument('-d', '--default', dest='default', action='store', type=str,
                        help='Select default param values instead of configured '
                        'values (implies --initial). Valid values: system|current_setup',
                        default=None)

    args = parser.parse_args()
    ulog_file_name = args.filename
    disable_str_exceptions = args.ignore

    message_filter = []
    if not args.initial: message_filter = None

    ulog = ULog(ulog_file_name, message_filter, disable_str_exceptions)

    params = ulog.initial_parameters
    if args.default is not None:
        params = get_defaults(ulog, args.default)
        args.initial = True

    param_keys = sorted(params.keys())
    delimiter = args.delimiter
    output_file = args.output_filename

    if args.format == "csv":
        for param_key in param_keys:
            output_file.write(param_key)
            if args.timestamps:
                output_file.write(delimiter)
                output_file.write(str(params[param_key]))
                for t, name, value in ulog.changed_parameters:
                    if name == param_key:
                        output_file.write(delimiter)
                        output_file.write(str(value))

                output_file.write('\n')
                output_file.write("timestamp")
                output_file.write(delimiter)
                output_file.write('0')
                for t, name, value in ulog.changed_parameters:
                    if name == param_key:
                        output_file.write(delimiter)
                        output_file.write(str(t))

                output_file.write('\n')
            else:
                output_file.write(delimiter)
                output_file.write(str(params[param_key]))
                if not args.initial:
                    for t, name, value in ulog.changed_parameters:
                        if name == param_key:
                            output_file.write(delimiter)
                            output_file.write(str(value))
                output_file.write('\n')

    elif args.format == "octave":

        for param_key in param_keys:
            output_file.write('# name ')
            output_file.write(param_key)
            values = [params[param_key]]

            if not args.initial:
                for t, name, value in ulog.changed_parameters:
                    if name == param_key:
                        values += [value]

            if len(values) > 1:
                output_file.write('\n# type: matrix\n')
                output_file.write('# rows: 1\n')
                output_file.write('# columns: ')
                output_file.write(str(len(values)) + '\n')
                for value in values:
                    output_file.write(str(value) + ' ')

            else:
                output_file.write('\n# type: scalar\n')
                output_file.write(str(values[0]))

            output_file.write('\n')

    elif args.format == "qgc":

        for param_key in param_keys:
            sys_id = 1
            comp_id = 1
            delimiter = '\t'
            param_value = params[param_key]

            output_file.write(str(sys_id))
            output_file.write(delimiter)
            output_file.write(str(comp_id))
            output_file.write(delimiter)
            output_file.write(param_key)
            output_file.write(delimiter)
            output_file.write(str(param_value))
            output_file.write(delimiter)

            if isinstance(param_value, float):
                # Float
                param_type = 9
            else:
                # Int
                param_type = 6

            output_file.write(str(param_type))
            output_file.write('\n')
