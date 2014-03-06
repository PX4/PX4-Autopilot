#!/usr/bin/env python3
#
# UAVCAN DSDL compiler for libuavcan
#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

import sys, os, argparse, logging
from mako.template import Template

RUNNING_FROM_SRC_DIR = os.path.abspath(__file__).endswith(os.path.join('libuavcan', 'dsdl_compiler', 'dsdlc.py'))
if RUNNING_FROM_SRC_DIR:
    print('Running from the source directory')
    scriptdir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(os.path.join(scriptdir, '..', '..', 'pyuavcan'))

from pyuavcan import dsdl

MAX_BITLEN_FOR_ENUM = 31
OUTPUT_FILE_EXTENSION = 'hpp'
OUTPUT_FILE_PERMISSIONS = 0o444  # Read only for all
TEMPLATE_FILENAME = os.path.join(os.path.dirname(__file__), 'data_type_template.hpp')

# -----------------

class DsdlCompilerException(Exception):
    pass

def pretty_filename(filename):
    a = os.path.abspath(filename)
    r = os.path.relpath(filename)
    return a if len(a) < len(r) else r

def type_output_filename(t):
    assert t.category == t.CATEGORY_COMPOUND
    return t.full_name.replace('.', os.path.sep) + '.' + OUTPUT_FILE_EXTENSION

def die(text):
    print(text, file=sys.stderr)
    exit(1)

def configure_logging(verbosity):
    fmt = '%(message)s'
    level = { 0: logging.WARNING, 1: logging.INFO, 2: logging.DEBUG }.get(verbosity or 0, logging.DEBUG)
    logging.basicConfig(stream=sys.stderr, level=level, format=fmt)

def run_parser(source_dir, search_dirs):
    try:
        types = dsdl.parse_namespace(source_dir, search_dirs)
    except dsdl.DsdlException as ex:
        logging.info('Parser failure', exc_info=True)
        die(str(ex))
    return types

def run_generator(types, dest_dir):
    try:
        dest_dir = os.path.abspath(dest_dir)  # Removing '..'
        os.makedirs(dest_dir, exist_ok=True)
        for t in types:
            logging.info('Generating type %s', t.full_name)
            filename = os.path.join(dest_dir, type_output_filename(t))
            text = generate_one_type(t)
            write_generated_data(filename, text)
    except Exception as ex:
        logging.info('Generator failure', exc_info=True)
        die(str(ex))

def write_generated_data(filename, data):
    dirname = os.path.dirname(filename)
    os.makedirs(dirname, exist_ok=True)

    # Lazy update - file will not be rewritten if its content is not going to change
    if os.path.exists(filename):
        with open(filename) as f:
            existing_data = f.read()
        if data == existing_data:
            logging.info('Up to date [%s]', pretty_filename(filename))
            return
        logging.info('Rewriting [%s]', pretty_filename(filename))
        os.remove(filename)
    else:
        logging.info('Creating [%s]', pretty_filename(filename))

    # Full rewrite
    with open(filename, 'w') as f:
        f.write(data)
    try:
        os.chmod(filename, OUTPUT_FILE_PERMISSIONS)
    except (OSError, IOError) as ex:
        logging.warning('Failed to set permissions for %s: %s', pretty_filename(filename), ex)

def type_to_cpp_type(t):
    if t.category == t.CATEGORY_PRIMITIVE:
        cast_mode = {
            t.CAST_MODE_SATURATED: '::uavcan::CastModeSaturate',
            t.CAST_MODE_TRUNCATED: '::uavcan::CastModeTruncate',
        }[t.cast_mode]
        if t.kind == t.KIND_FLOAT:
            return '::uavcan::FloatSpec< %d, %s >' % (t.bitlen, cast_mode)
        else:
            signedness = {
                t.KIND_BOOLEAN: '::uavcan::SignednessUnsigned',
                t.KIND_UNSIGNED_INT: '::uavcan::SignednessUnsigned',
                t.KIND_SIGNED_INT: '::uavcan::SignednessSigned',
            }[t.kind]
            return '::uavcan::IntegerSpec< %d, %s, %s >' % (t.bitlen, signedness, cast_mode)
    elif t.category == t.CATEGORY_ARRAY:
        value_type = type_to_cpp_type(t.value_type)
        mode = {
            t.MODE_STATIC: '::uavcan::ArrayModeStatic',
            t.MODE_DYNAMIC: '::uavcan::ArrayModeDynamic',
        }[t.mode]
        return '::uavcan::Array< %s, %s, %d >' % (value_type, mode, t.max_size)
    elif t.category == t.CATEGORY_COMPOUND:
        return '::' + t.full_name.replace('.', '::')
    else:
        raise DsdlCompilerException('Unknown type category: %s' % t.category)

def generate_one_type(t):
    t.short_name = t.full_name.split('.')[-1]
    t.cpp_type_name = t.short_name + '_'

    # Dependencies (no duplicates)
    def fields_includes(fields):
        return set(type_output_filename(x.type) for x in fields if x.type.category == x.type.CATEGORY_COMPOUND)

    if t.kind == t.KIND_MESSAGE:
        t.cpp_includes = fields_includes(t.fields)
    else:
        t.cpp_includes = fields_includes(t.request_fields + t.response_fields)

    t.cpp_namespace_components = t.full_name.split('.')[:-1]
    t.has_default_dtid = t.default_dtid is not None

    # Attribute types
    def inject_cpp_types(attributes):
        for a in attributes:
            a.cpp_type = type_to_cpp_type(a.type)

    if t.kind == t.KIND_MESSAGE:
        inject_cpp_types(t.fields)
        inject_cpp_types(t.constants)
    else:
        inject_cpp_types(t.request_fields)
        inject_cpp_types(t.request_constants)
        inject_cpp_types(t.response_fields)
        inject_cpp_types(t.response_constants)

    # Constant properties
    def inject_constant_info(constants):
        for c in constants:
            if c.type.kind == c.type.KIND_FLOAT:
                c.cpp_use_enum = False
                float(c.string_value)  # Making sure that this is a valid float literal
                c.cpp_value = c.string_value
            else:
                int(c.string_value)  # Making sure that this is a valid integer literal
                c.cpp_use_enum = c.value >= 0 and c.type.bitlen <= MAX_BITLEN_FOR_ENUM
                c.cpp_value = c.string_value
                if c.type.kind == c.type.KIND_UNSIGNED_INT:
                    c.cpp_value += 'U'

    if t.kind == t.KIND_MESSAGE:
        inject_constant_info(t.constants)
    else:
        inject_constant_info(t.request_constants)
        inject_constant_info(t.response_constants)

    # Data type kind
    t.cpp_kind = {
        t.KIND_MESSAGE: '::uavcan::DataTypeKindMessage',
        t.KIND_SERVICE: '::uavcan::DataTypeKindService',
    }[t.kind]

    # Generation
    template = Template(filename=TEMPLATE_FILENAME)
    text = template.render(t=t)
    text = '\n'.join(x.rstrip() for x in text.splitlines())
    text = text.replace('\n\n\n\n', '\n\n').replace('\n\n\n', '\n\n')
    text = text.replace('{\n\n ', '{\n ')
    return text

# -----------------

DESCRIPTION = '''UAVCAN DSDL compiler. Takes an input directory that contains an hierarchy of DSDL
definitions and converts it into compatible hierarchy of C++ types for libuavcan.'''

DEFAULT_OUTDIR = './dsdlc_output'

argparser = argparse.ArgumentParser(description=DESCRIPTION)
argparser.add_argument('source_dir', nargs='+', help='source directory with DSDL definitions')
argparser.add_argument('--verbose', '-v', action='count', help='verbosity level (-v, -vv)')
argparser.add_argument('--outdir', '-O', default=DEFAULT_OUTDIR, help='output directory, default %s' % DEFAULT_OUTDIR)
argparser.add_argument('--incdir', '-I', default=[], action='append', help='nested type namespaces')
args = argparser.parse_args()

configure_logging(args.verbose)

types = []
for srcdir in args.source_dir:
    new_types = run_parser(srcdir, args.incdir + args.source_dir)
    types += new_types
    logging.info('%d types from [%s]', len(new_types), srcdir)

if not types:
    die('No type definitions were found')

run_generator(types, args.outdir)
