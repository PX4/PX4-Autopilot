#! /usr/bin/env python3
""" Script to validate JSON file(s) against a schema file according to
https://json-schema.org/"""

import argparse
import sys
import json
import os

try:
    from jsonschema import validate
except ImportError as e:
    print("Failed to import jsonschema: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user jsonschema")
    print("")
    sys.exit(1)


parser = argparse.ArgumentParser(description='Validate JSON file(s) against a schema')
parser.add_argument('json_file', nargs='+', help='JSON config file(s)')
parser.add_argument('--schema-file', type=str, action='store',
                    help='JSON schema file', required=True)
parser.add_argument('--skip-if-no-schema', dest='skip_if_no_schema', action='store_true',
                    help='Skip test if schema file does not exist')
parser.add_argument('--version-minimum', dest='version_minimum', type=int,
                    help='Replace the schema minimum of the top-level version field')
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()
schema_file = args.schema_file
json_files = args.json_file
verbose = args.verbose

if args.skip_if_no_schema and not os.path.isfile(schema_file):
    sys.exit(0)

# load the schema
with open(schema_file, 'r') as stream:
    schema = json.load(stream)

if args.version_minimum is not None:
    properties = schema.get('properties') if isinstance(schema, dict) else None
    version_schema = properties.get('version') if isinstance(properties, dict) else None
    if not isinstance(version_schema, dict) or 'minimum' not in version_schema:
        print("Schema {:} has no top-level version minimum".format(schema_file))
        sys.exit(1)
    version_schema['minimum'] = args.version_minimum

# validate files
for json_file in json_files:
    if verbose: print("Validating {:}".format(json_file))
    with open(json_file, 'r') as stream:
        json_data = json.load(stream)

    try:
        validate(instance=json_data, schema=schema)
    except:
        print("JSON validation for {:} failed (schema: {:})".format(json_file, schema_file))
        raise
