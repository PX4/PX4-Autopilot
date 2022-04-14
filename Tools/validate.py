#! /usr/bin/env python3
""" Script to validate YAML file(s) against a YAML schema file """

import argparse
import logging
import pprint
import sys
import os

try:
    import yaml
except ImportError as e:
    print("Failed to import yaml: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyyaml")
    print("")
    sys.exit(1)

try:
    import cerberus
except ImportError as e:
    print("Failed to import cerberus: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user cerberus")
    print("")
    sys.exit(1)


def load_data_file(file_name: str):
    with open(file_name) as stream:
        try:
            return yaml.safe_load(stream)
        except:
            raise Exception("Unable to parse schema file: syntax error or unsupported file format")


def main():
    parser = argparse.ArgumentParser(description='Validate YAML or JSON file(s) against a schema')

    parser.add_argument('data_file', nargs='+', help='data file(s)')
    parser.add_argument('--schema-file', type=str, action='store',
                        help='schema file', required=True)
    parser.add_argument('--skip-if-no-schema', dest='skip_if_no_schema',
                        action='store_true',
                        help='Skip test if schema file does not exist')
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                        help='Verbose Output')

    args = parser.parse_args()

    data_files = args.data_file
    schema_file = args.schema_file

    if args.verbose:
        logging.basicConfig(level=logging.INFO)
    else:
        logging.basicConfig(level=logging.ERROR)

    if args.skip_if_no_schema and not os.path.isfile(schema_file):
        logging.info(f"Schema file {schema_file} not found, skipping")
        sys.exit(0)

    # load the schema
    schema = load_data_file(schema_file)
    validator = cerberus.Validator(schema, allow_unknown=False)

    # validate the specified data files
    for data_file in data_files:
        logging.info(f"Validating {data_file}")

        document = load_data_file(data_file)

        # ignore top-level entries prefixed with __ for yaml module definitions
        for key in list(document.keys()):
            if key.startswith('__'):
                del document[key]

        if not validator.validate(document):
            logging.error(f"Found validation errors with {data_file}:")
            logging.error(pprint.pformat(validator.errors))

            raise Exception("Validation of {:} failed".format(data_file))


if __name__ == "__main__":
    main()
