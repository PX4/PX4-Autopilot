#
# UAVCAN DSDL file parser
#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
import os, re, logging
from io import StringIO
from .signature import compute_signature
from .common import DsdlException, pretty_filename
from .type_limits import get_unsigned_integer_range, get_signed_integer_range, get_float_range

# Python 2.7 compatibility
try:
    str = unicode
except NameError:
    pass
try:
    long(1)
except NameError:
    long = int

MAX_FULL_TYPE_NAME_LEN = 80
DATA_TYPE_ID_MAX = 1023
MAX_DATA_STRUCT_LEN_BYTES = 439

class Type:
    '''
    Common type description. The specialized type description classes inherit from this one.
    Fields:
        full_name    Full type name string, e.g. "uavcan.protocol.NodeStatus"
        category     Any CATEGORY_*
    '''
    CATEGORY_PRIMITIVE = 0
    CATEGORY_ARRAY = 1
    CATEGORY_COMPOUND = 2

    def __init__(self, full_name, category):
        self.full_name = str(full_name)
        self.category = category

    def __str__(self):
        return self.get_normalized_definition()

    __repr__ = __str__

class PrimitiveType(Type):
    '''
    Primitive type description, e.g. bool or float16.
    Fields:
        kind         Any KIND_*
        bitlen       Bit length, 1 to 64
        cast_mode    Any CAST_MODE_*
        value_range  Tuple containing min and max values: (min, max)
    '''
    KIND_BOOLEAN = 0
    KIND_UNSIGNED_INT = 1
    KIND_SIGNED_INT = 2
    KIND_FLOAT = 3

    CAST_MODE_SATURATED = 0
    CAST_MODE_TRUNCATED = 1

    def __init__(self, kind, bitlen, cast_mode):
        self.kind = kind
        self.bitlen = bitlen
        self.cast_mode = cast_mode
        Type.__init__(self, self.get_normalized_definition(), Type.CATEGORY_PRIMITIVE)
        self.value_range = {
            PrimitiveType.KIND_BOOLEAN: get_unsigned_integer_range,
            PrimitiveType.KIND_UNSIGNED_INT: get_unsigned_integer_range,
            PrimitiveType.KIND_SIGNED_INT: get_signed_integer_range,
            PrimitiveType.KIND_FLOAT: get_float_range
        }[self.kind](bitlen)

    def get_normalized_definition(self):
        '''Please refer to the specification for details about normalized definitions.'''
        cast_mode = 'saturated' if self.cast_mode == PrimitiveType.CAST_MODE_SATURATED else 'truncated'
        primary_type = {
            PrimitiveType.KIND_BOOLEAN: 'bool',
            PrimitiveType.KIND_UNSIGNED_INT: 'uint' + str(self.bitlen),
            PrimitiveType.KIND_SIGNED_INT: 'int' + str(self.bitlen),
            PrimitiveType.KIND_FLOAT: 'float' + str(self.bitlen)
        }[self.kind]
        return cast_mode + ' ' + primary_type

    def validate_value_range(self, value):
        '''Checks value range, throws DsdlException if the value cannot be represented by this type.'''
        low, high = self.value_range
        if not low <= value <= high:
            error('Value [%s] is out of range %s', value, self.value_range)

    def get_max_bitlen(self):
        '''Returns type bit length.'''
        return self.bitlen

class ArrayType(Type):
    '''
    Array type description, e.g. float32[8], uint12[<34].
    Fields:
        value_type    Description of the array value type; the type of this field inherits Type, e.g. PrimitiveType
        mode          Any MODE_*
        max_size      Maximum number of elements in the array
    '''
    MODE_STATIC = 0
    MODE_DYNAMIC = 1

    def __init__(self, value_type, mode, max_size):
        self.value_type = value_type
        self.mode = mode
        self.max_size = max_size
        Type.__init__(self, self.get_normalized_definition(), Type.CATEGORY_ARRAY)

    def get_normalized_definition(self):
        '''Please refer to the specification for details about normalized definitions.'''
        typedef = self.value_type.get_normalized_definition()
        return ('%s[<=%d]' if self.mode == ArrayType.MODE_DYNAMIC else '%s[%d]') % (typedef, self.max_size)

    def get_max_bitlen(self):
        '''Returns total maximum bit length of the array, including length field if applicable.'''
        payload_max_bitlen = self.max_size * self.value_type.get_max_bitlen()
        return {
            self.MODE_DYNAMIC: payload_max_bitlen + self.max_size.bit_length(),
            self.MODE_STATIC: payload_max_bitlen
        }[self.mode]

class CompoundType(Type):
    '''
    Compound type description, e.g. uavcan.protocol.NodeStatus.
    Fields:
        source_file         Path to the DSDL definition file for this type
        default_dtid        Default Data Type ID, if specified, None otherwise
        kind                Any KIND_*
        source_text         Raw DSDL definition text (as is, with comments and the original formatting)
    
    Fields if kind == KIND_SERVICE:
        request_fields      Request struct field list, the type of each element is Field
        response_fields     Response struct field list
        request_constants   Request struct constant list, the type of each element is Constant
        response_constants  Response struct constant list
    
    Fields if kind == KIND_MESSAGE:
        fields              Field list, the type of each element is Field
        constants           Constant list, the type of each element is Constant
    
    Extra methods if kind == KIND_SERVICE:
        get_max_bitlen_request()    Returns maximum total bit length for the serialized request struct
        get_max_bitlen_response()   Same for the response struct
    
    Extra methods if kind == KIND_MESSAGE:
        get_max_bitlen()            Returns maximum total bit length for the serialized struct
    '''
    KIND_SERVICE = 0
    KIND_MESSAGE = 1

    def __init__(self, full_name, kind, source_file, default_dtid, source_text):
        Type.__init__(self, full_name, Type.CATEGORY_COMPOUND)
        self.source_file = source_file
        self.default_dtid = default_dtid
        self.kind = kind
        self.source_text = source_text
        max_bitlen_sum = lambda fields: sum([x.type.get_max_bitlen() for x in fields])
        if kind == CompoundType.KIND_SERVICE:
            self.request_fields = []
            self.response_fields = []
            self.request_constants = []
            self.response_constants = []
            self.get_max_bitlen_request = lambda: max_bitlen_sum(self.request_fields)
            self.get_max_bitlen_response = lambda: max_bitlen_sum(self.response_fields)
        elif kind == CompoundType.KIND_MESSAGE:
            self.fields = []
            self.constants = []
            self.get_max_bitlen = lambda: max_bitlen_sum(self.fields)
        else:
            error('Compound type of unknown kind [%s]', kind)

    def get_dsdl_signature_source_definition(self):
        '''
        Returns normalized DSDL definition text.
        Please refer to the specification for details about normalized DSDL definitions.
        '''
        txt = StringIO()
        txt.write(self.full_name + '\n')
        adjoin = lambda attrs: txt.write('\n'.join(x.get_normalized_definition() for x in attrs) + '\n')
        const_sort_key = lambda x: x.get_normalized_definition()
        if self.kind == CompoundType.KIND_SERVICE:
            adjoin(self.request_fields)
            adjoin(sorted(self.request_constants, key=const_sort_key))
            txt.write('\n---\n')
            adjoin(self.response_fields)
            adjoin(sorted(self.response_constants, key=const_sort_key))
        elif self.kind == CompoundType.KIND_MESSAGE:
            adjoin(self.fields)
            adjoin(sorted(self.constants, key=const_sort_key))
        else:
            error('Compound type of unknown kind [%s]', self.kind)
        return txt.getvalue().strip().replace('\n\n\n', '\n').replace('\n\n', '\n')

    def get_dsdl_signature(self):
        '''
        Computes DSDL signature of this type.
        Please refer to the specification for details about signatures.
        '''
        return compute_signature(self.get_dsdl_signature_source_definition())

    def get_normalized_definition(self):
        '''Returns full type name string, e.g. "uavcan.protocol.NodeStatus"'''
        return self.full_name


class Attribute:
    '''
    Base class of an attribute description.
    Fields:
        type    Attribute type description, the type of this field inherits the class Type, e.g. PrimitiveType
        name    Attribute name string
    '''
    def __init__(self, type, name):  # @ReservedAssignment
        self.type = type
        self.name = name

    def __str__(self):
        return self.get_normalized_definition()

    __repr__ = __str__

class Field(Attribute):
    '''
    Field description.
    Does not add new fields to Attribute.
    '''
    def get_normalized_definition(self):
        return '%s %s' % (self.type.get_normalized_definition(), self.name)

class Constant(Attribute):
    '''
    Constant description.
    Fields:
        init_expression    Constant initialization expression string, e.g. "2+2" or "'\x66'"
        value              Computed result of the initialization expression in the final type (e.g. int, float)
        string_value       Computed result of the initialization expression as string
    '''
    def __init__(self, type, name, init_expression, value):  # @ReservedAssignment
        Attribute.__init__(self, type, name)
        self.init_expression = init_expression
        self.value = value
        self.string_value = repr(value)
        if isinstance(value, long):
            self.string_value = self.string_value.replace('L', '')

    def get_normalized_definition(self):
        return '%s %s = %s' % (self.type.get_normalized_definition(), self.name, self.init_expression)


class Parser:
    '''
    DSDL parser logic. Do not use this class directly; use the helper function instead.
    '''
    LOGGER_NAME = 'dsdl_parser'

    def __init__(self, search_dirs):
        self.search_dirs = validate_search_directories(search_dirs)
        self.log = logging.getLogger(Parser.LOGGER_NAME)

    def _namespace_from_filename(self, filename):
        search_dirs = sorted(map(os.path.abspath, self.search_dirs))  # Nested last
        filename = os.path.abspath(filename)
        for dirname in search_dirs:
            root_ns = dirname.split(os.path.sep)[-1]
            if filename.startswith(dirname):
                dir_len = len(dirname)
                basename_len = len(os.path.basename(filename))
                ns = filename[dir_len:-basename_len]
                ns = (root_ns + '.' + ns.replace(os.path.sep, '.').strip('.')).strip('.')
                validate_namespace_name(ns)
                return ns
        error('File [%s] was not found in search directories', filename)

    def _full_typename_and_dtid_from_filename(self, filename):
        basename = os.path.basename(filename)
        items = basename.split('.')
        if (len(items) != 2 and len(items) != 3) or items[-1] != 'uavcan':
            error('Invalid file name [%s]; expected pattern: [<default-dtid>.]<short-type-name>.uavcan', basename)
        if len(items) == 2:
            default_dtid, name = None, items[0]
        else:
            default_dtid, name = items[0], items[1]
            try:
                default_dtid = int(default_dtid)
            except ValueError:
                error('Invalid default data type ID [%s]', default_dtid)
            validate_data_type_id(default_dtid)
        full_name = self._namespace_from_filename(filename) + '.' + name
        validate_compound_type_full_name(full_name)
        return full_name, default_dtid

    def _locate_compound_type_definition(self, referencing_filename, typename):
        def locate_namespace_directory(namespace):
            namespace_components = namespace.split('.')
            root_namespace, sub_namespace_components = namespace_components[0], namespace_components[1:]
            for directory in self.search_dirs:
                if directory.split(os.path.sep)[-1] == root_namespace:
                    return os.path.join(directory, *sub_namespace_components)
            error('Unknown namespace [%s]', namespace)

        if '.' not in typename:
            current_namespace = self._namespace_from_filename(referencing_filename)
            full_typename = current_namespace + '.' + typename
        else:
            full_typename = typename
        namespace = '.'.join(full_typename.split('.')[:-1])
        directory = locate_namespace_directory(namespace)
        self.log.debug('Searching for [%s] in [%s]', full_typename, directory)

        for fn in os.listdir(directory):
            fn = os.path.join(directory, fn)
            if os.path.isfile(fn):
                try:
                    fn_full_typename, _dtid = self._full_typename_and_dtid_from_filename(fn)
                    if full_typename == fn_full_typename:
                        return fn
                except Exception as ex:
                    self.log.debug('Unknown file [%s], skipping... [%s]', pretty_filename(fn), ex)
        error('Type definition not found [%s]', typename)

    def _parse_array_type(self, filename, value_typedef, size_spec, cast_mode):
        self.log.debug('Parsing the array value type [%s]...', value_typedef)
        value_type = self._parse_type(filename, value_typedef, cast_mode)
        enforce(value_type.category != value_type.CATEGORY_ARRAY,
                 'Multidimensional arrays are not allowed (protip: use nested types)')
        try:
            if size_spec.startswith('<='):
                max_size = int(size_spec[2:], 0)
                mode = ArrayType.MODE_DYNAMIC
            elif size_spec.startswith('<'):
                max_size = int(size_spec[1:], 0) - 1
                mode = ArrayType.MODE_DYNAMIC
            else:
                max_size = int(size_spec, 0)
                mode = ArrayType.MODE_STATIC
        except ValueError:
            error('Invalid array size specifier [%s] (valid patterns: [<=X], [<X], [X])', size_spec)
        enforce(max_size > 0, 'Array size must be positive, not %d', max_size)
        return ArrayType(value_type, mode, max_size)

    def _parse_primitive_type(self, filename, base_name, bitlen, cast_mode):
        if cast_mode is None or cast_mode == 'saturated':
            cast_mode = PrimitiveType.CAST_MODE_SATURATED
        elif cast_mode == 'truncated':
            cast_mode = PrimitiveType.CAST_MODE_TRUNCATED
        else:
            error('Invalid cast mode [%s]', cast_mode)

        if base_name == 'bool':
            return PrimitiveType(PrimitiveType.KIND_BOOLEAN, 1, cast_mode)
        try:
            kind = {
                'uint' : PrimitiveType.KIND_UNSIGNED_INT,
                'int'  : PrimitiveType.KIND_SIGNED_INT,
                'float': PrimitiveType.KIND_FLOAT,
            }[base_name]
        except KeyError:
            error('Unknown primitive type (note: compound types should be in CamelCase)')

        if kind == PrimitiveType.KIND_FLOAT:
            enforce(bitlen in (16, 32, 64), 'Invalid bit length for float type [%d]', bitlen)
        else:
            enforce(2 <= bitlen <= 64, 'Invalid bit length [%d] (note: use bool instead of uint1)', bitlen)
        return PrimitiveType(kind, bitlen, cast_mode)

    def _parse_compound_type(self, filename, typedef):
        definition_filename = self._locate_compound_type_definition(filename, typedef)
        self.log.debug('Nested type [%s] is defined in [%s], parsing...', typedef, pretty_filename(definition_filename))
        t = self.parse(definition_filename)
        if t.kind == t.KIND_SERVICE:
            error('A service type can not be nested into another compound type')
        return t

    def _parse_type(self, filename, typedef, cast_mode):
        typedef = typedef.strip()
        array_match = re.match(r'(.+?)\[([^\]]*)\]$', typedef)
        primitive_match = re.match(r'([a-z]+)(\d{1,2})$|(bool)$', typedef)

        if array_match:
            assert not primitive_match
            value_typedef = array_match.group(1).strip()
            size_spec = array_match.group(2).strip()
            return self._parse_array_type(filename, value_typedef, size_spec, cast_mode)
        elif primitive_match:
            if primitive_match.group(0) == 'bool':
                return self._parse_primitive_type(filename, 'bool', 1, cast_mode)
            else:
                base_name = primitive_match.group(1)
                bitlen = int(primitive_match.group(2))
                return self._parse_primitive_type(filename, base_name, bitlen, cast_mode)
        else:
            enforce(cast_mode is None, 'Cast mode specifier is not applicable for compound types [%s]', cast_mode)
            return self._parse_compound_type(filename, typedef)

    def _make_constant(self, attrtype, name, init_expression):
        enforce(attrtype.category == attrtype.CATEGORY_PRIMITIVE, 'Invalid type for constant')
        init_expression = ''.join(init_expression.split())  # Remove spaces
        value = evaluate_expression(init_expression)

        if isinstance(value, str) and len(value) == 1:  # ASCII character
            value = ord(value)
        elif isinstance(value, (float, int, bool, long)):  # Numeric literal
            value = {
                attrtype.KIND_UNSIGNED_INT : long,
                attrtype.KIND_SIGNED_INT : long,
                attrtype.KIND_BOOLEAN : int,  # Not bool because we need to check range
                attrtype.KIND_FLOAT : float
            }[attrtype.kind](value)
        else:
            error('Invalid type of constant initialization expression [%s]', type(value).__name__)

        self.log.debug('Constant initialization expression evaluated as: [%s] --> %s', init_expression, repr(value))
        attrtype.validate_value_range(value)
        return Constant(attrtype, name, init_expression, value)

    def _parse_line(self, filename, tokens):
        cast_mode = None
        if tokens[0] == 'saturated' or tokens[0] == 'truncated':
            cast_mode, tokens = tokens[0], tokens[1:]

        if len(tokens) < 2:
            error('Invalid attribute definition')

        typename, attrname, tokens = tokens[0], tokens[1], tokens[2:]
        validate_attribute_name(attrname)
        attrtype = self._parse_type(filename, typename, cast_mode)

        if len(tokens) > 0:
            if len(tokens) < 2 or tokens[0] != '=':
                error('Constant assignment expected')
            expression = ' '.join(tokens[1:])
            return self._make_constant(attrtype, attrname, expression)
        else:
            return Field(attrtype, attrname)

    def _tokenize(self, text):
        for idx, line in enumerate(text.splitlines()):
            line = re.sub('#.*', '', line).strip()  # Remove comments and leading/trailing whitespaces
            if line:
                tokens = [tk for tk in line.split() if tk]
                yield idx + 1, tokens

    def parse(self, filename):
        try:
            filename = os.path.abspath(filename)
            with open(filename) as f:
                source_text = f.read()

            full_typename, default_dtid = self._full_typename_and_dtid_from_filename(filename)
            numbered_lines = list(self._tokenize(source_text))
            all_attributes_names = set()
            fields, constants, resp_fields, resp_constants = [], [], [], []
            response_part = False
            for num, tokens in numbered_lines:
                if tokens == ['---']:
                    response_part = True
                    all_attributes_names = set()
                    continue
                try:
                    attr = self._parse_line(filename, tokens)
                    if attr.name in all_attributes_names:
                        error('Duplicated attribute name [%s]', attr.name)
                    all_attributes_names.add(attr.name)
                    if isinstance(attr, Constant):
                        (resp_constants if response_part else constants).append(attr)
                    elif isinstance(attr, Field):
                        (resp_fields if response_part else fields).append(attr)
                    else:
                        error('Unknown attribute type - internal error')
                except DsdlException as ex:
                    if not ex.line:
                        ex.line = num
                    raise ex
                except Exception as ex:
                    self.log.error('Internal error', exc_info=True)
                    raise DsdlException('Internal error: %s' % str(ex), line=num)

            if response_part:
                t = CompoundType(full_typename, CompoundType.KIND_SERVICE, filename, default_dtid, source_text)
                t.request_fields = fields
                t.request_constants = constants
                t.response_fields = resp_fields
                t.response_constants = resp_constants
                max_bitlen = t.get_max_bitlen_request(), t.get_max_bitlen_response()
                max_bytelen = tuple(map(bitlen_to_bytelen, max_bitlen))
            else:
                t = CompoundType(full_typename, CompoundType.KIND_MESSAGE, filename, default_dtid, source_text)
                t.fields = fields
                t.constants = constants
                max_bitlen = t.get_max_bitlen()
                max_bytelen = bitlen_to_bytelen(max_bitlen)

            validate_data_struct_len(t)
            self.log.info('Type [%s], default DTID: %s, signature: %08x, maxbits: %s, maxbytes: %s, DSSD:',
                          full_typename, default_dtid, t.get_dsdl_signature(), max_bitlen, max_bytelen)
            for ln in t.get_dsdl_signature_source_definition().splitlines():
                self.log.info('    %s', ln)
            return t
        except DsdlException as ex:
            if not ex.file:
                ex.file = filename
            raise ex
        except IOError as ex:
            raise DsdlException('IO error: %s' % str(ex), file=filename)
        except Exception as ex:
            self.log.error('Internal error', exc_info=True)
            raise DsdlException('Internal error: %s' % str(ex), file=filename)


def error(fmt, *args):
    raise DsdlException(fmt % args)

def enforce(cond, fmt, *args):
    if not cond:
        error(fmt, *args)

def bitlen_to_bytelen(x):
    return int((x + 7) / 8)

def evaluate_expression(expression):
    try:
        env = {
            'locals': None,
            'globals': None,
            '__builtins__': None,
            'true': 1,
            'false': 0
        }
        return eval(expression, env)
    except Exception as ex:
        error('Cannot evaluate expression: %s', str(ex))

def validate_search_directories(dirnames):
    dirnames = set(dirnames)
    dirnames = list(map(os.path.abspath, dirnames))
    for d1 in dirnames:
        for d2 in dirnames:
            if d1 == d2:
                continue
            enforce(not d1.startswith(d2), 'Nested search directories are not allowed [%s] [%s]', d1, d2)
            enforce(d1.split(os.path.sep)[-1] != d2.split(os.path.sep)[-1],
                     'Namespace roots must be unique [%s] [%s]', d1, d2)
    return dirnames

def validate_namespace_name(name):
    for component in name.split('.'):
        enforce(re.match(r'[a-z][a-z0-9_]*$', component), 'Invalid namespace name [%s]', name)
    enforce(len(name) <= MAX_FULL_TYPE_NAME_LEN, 'Namespace name is too long [%s]', name)

def validate_compound_type_full_name(name):
    enforce('.' in name, 'Full type name must explicitly specify its namespace [%s]', name)
    short_name = name.split('.')[-1]
    namespace = '.'.join(name.split('.')[:-1])
    validate_namespace_name(namespace)
    enforce(re.match(r'[A-Z][A-Za-z0-9_]*$', short_name), 'Invalid type name [%s]', name)
    enforce(len(name) <= MAX_FULL_TYPE_NAME_LEN, 'Type name is too long [%s]', name)

def validate_attribute_name(name):
    enforce(re.match(r'[a-zA-Z][a-zA-Z0-9_]*$', name), 'Invalid attribute name [%s]', name)

def validate_data_type_id(dtid):
    enforce(0 <= dtid <= DATA_TYPE_ID_MAX, 'Invalid data type ID [%s]', dtid)

def validate_data_struct_len(t):
    enforce(t.category == t.CATEGORY_COMPOUND, 'Data structure length can be enforced only for compound types')
    if t.kind == t.KIND_MESSAGE:
        bitlens = [t.get_max_bitlen()]
    elif t.kind == t.KIND_SERVICE:
        bitlens = t.get_max_bitlen_request(), t.get_max_bitlen_response()
    for bitlen in bitlens:
        bytelen = bitlen_to_bytelen(bitlen)
        enforce(0 <= bytelen <= MAX_DATA_STRUCT_LEN_BYTES,
                 'Max data structure length is invalid: %d bits, %d bytes', bitlen, bytelen)


def parse_namespaces(source_dirs, search_dirs=None):
    '''
    Use only this function to parse DSDL definitions.
    This function takes a list of root namespace directories (containing DSDL definition files to parse) and an
    optional list of search directories (containing DSDL definition files that can be referenced from the types
    that are going to be parsed).
    
    Returns the list of parsed type definitions, where type of each element is CompoundType.
    
    Args:
        source_dirs    List of root namespace directories to parse.
        search_dirs    List of root namespace directories with referenced types (optional). This list is
                       automaitcally extended with source_dirs.
    Example:
        >>> import pyuavcan
        >>> a = pyuavcan.dsdl.parse_namespaces(['../dsdl/uavcan'])
        >>> len(a)
        77
        >>> a[0]
        uavcan.Timestamp
        >>> a[0].fields
        [truncated uint48 husec]
        >>> a[0].constants
        [saturated uint48 UNKNOWN = 0, saturated uint48 USEC_PER_LSB = 100]
    '''
    def walk():
        import fnmatch
        from functools import partial
        def on_walk_error(directory, ex):
            raise DsdlException('OS error in [%s]: %s' % (directory, str(ex)))
        for source_dir in source_dirs:
            walker = os.walk(source_dir, onerror=partial(on_walk_error, source_dir), followlinks=True)
            for root, _dirnames, filenames in walker:
                for filename in fnmatch.filter(filenames, '*.uavcan'):
                    filename = os.path.join(root, filename)
                    yield filename

    all_default_dtid = {}  # (kind, dtid) : filename
    def ensure_unique_dtid(t, filename):
        if t.default_dtid is None:
            return
        key = t.kind, t.default_dtid
        if key in all_default_dtid:
            first = pretty_filename(all_default_dtid[key])
            second = pretty_filename(filename)
            error('Default data type ID collision: [%s] [%s]', first, second)
        all_default_dtid[key] = filename

    parser = Parser(source_dirs + (search_dirs or []))
    output_types = []
    for filename in walk():
        t = parser.parse(filename)
        ensure_unique_dtid(t, filename)
        output_types.append(t)
    return output_types


if __name__ == '__main__':
    import sys
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(levelname)s: %(message)s')

    if not sys.argv[1:]:
        self_directory = os.path.dirname(__file__)
        test_dir = os.path.join(self_directory, '..', '..', '..', 'dsdl', 'uavcan')
        t = parse_namespaces([test_dir], [])
        print(len(t))
    else:
        t = parse_namespaces([sys.argv[1]], sys.argv[2:])
        print(len(t))
