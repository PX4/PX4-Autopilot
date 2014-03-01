#
# UAVCAN DSDL file parser
#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

import os, re, logging
from .signature import compute_signature

MAX_FULL_TYPE_NAME_LEN = 80

class ParserException(Exception):
    def __init__(self, text, *args, file=None, line=None):
        super().__init__(text, *args)
        self.file = file
        self.line = line

    def __str__(self):
        if self.file and self.line:
            return '%s:%s: %s' % (_pretty_filename(self.file), self.line, super().__str__())
        if self.file:
            return '%s:?: %s' % (_pretty_filename(self.file), super().__str__())
        return super().__str__()

class Type:
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
    KIND_BOOLEAN = 0
    KIND_UNSIGNED_INT = 1
    KIND_SIGNED_INT = 2
    KIND_FLOAT = 3

    CAST_MODE_SATURATED = 0
    CAST_MODE_TRUNCATED = 1

    def __init__(self, kind, bit_len, cast_mode):
        self.kind = kind
        self.bit_len = bit_len
        self.cast_mode = cast_mode
        super().__init__(self.get_normalized_definition(), Type.CATEGORY_PRIMITIVE)

    def get_normalized_definition(self):
        cast_mode = 'saturated' if self.cast_mode == PrimitiveType.CAST_MODE_SATURATED else 'truncated'
        if self.kind == PrimitiveType.KIND_BOOLEAN:
            primary_type = 'bool'
        elif self.kind == PrimitiveType.KIND_UNSIGNED_INT:
            primary_type = 'uint' + str(self.bit_len)
        elif self.kind == PrimitiveType.KIND_SIGNED_INT:
            primary_type = 'int' + str(self.bit_len)
        elif self.kind == PrimitiveType.KIND_FLOAT:
            primary_type = 'float' + str(self.bit_len)
        else:
            raise ParserException('Primitive type of unknown kind', self.kind)
        return cast_mode + ' ' + primary_type

class ArrayType(Type):
    MODE_STATIC = 0
    MODE_DYNAMIC = 1

    def __init__(self, value_type, mode, max_size):
        self.value_type = value_type
        self.mode = mode
        self.max_size = max_size
        super().__init__(self.get_normalized_definition(), Type.CATEGORY_ARRAY)

    def get_normalized_definition(self):
        typedef = self.value_type.get_normalized_definition()
        return ('%s[<=%d]' if self.mode == ArrayType.MODE_DYNAMIC else '%s[%d]') % (typedef, self.max_size)

class CompoundType(Type):
    KIND_SERVICE = 0
    KIND_MESSAGE = 1

    def __init__(self, full_name, kind, dsdl_signature, dsdl_path, default_dtid):
        super().__init__(full_name, Type.CATEGORY_COMPOUND)
        self.dsdl_signature = dsdl_signature
        self.dsdl_path = dsdl_path
        self.default_dtid = default_dtid
        self.kind = kind
        if kind == CompoundType.KIND_SERVICE:
            self.request_fields = []
            self.response_fields = []
            self.request_constants = []
            self.response_constants = []
        elif kind == CompoundType.KIND_MESSAGE:
            self.fields = []
            self.constants = []
        else:
            raise ParserException('Compound type of unknown kind', kind)

    def get_normalized_definition(self):
        return self.full_name

    def get_normalized_attributes_definitions(self):
        def normdef(attrs):
            return '\n'.join([x.get_normalized_definition() for x in attrs])
        if self.kind == self.KIND_MESSAGE:
            return '\n'.join([normdef(self.fields), normdef(self.constants)])
        elif self.kind == self.KIND_SERVICE:
            rq = '\n'.join([normdef(self.request_fields), normdef(self.request_constants)])
            rs = '\n'.join([normdef(self.response_fields), normdef(self.response_constants)])
            return (rq + '\n---\n' + rs).strip('\n').replace('\n\n', '\n')


class Attribute:
    def __init__(self, type, name):  # @ReservedAssignment
        self.type = type
        self.name = name

    def __str__(self):
        return self.get_normalized_definition()

    __repr__ = __str__

class Field(Attribute):
    def get_normalized_definition(self):
        return '%s %s' % (self.type.get_normalized_definition(), self.name)

class Constant(Attribute):
    def __init__(self, type, name, init_expression):  # @ReservedAssignment
        super().__init__(type, name)
        self.init_expression = init_expression

    def get_normalized_definition(self):
        return '%s %s = %s' % (self.type.get_normalized_definition(), self.name, self.init_expression)

    def evaluate_init_expression(self):
        return evaluate_expression(self.init_expression)


def _enforce(cond, *exception_args):
    if not cond:
        raise ParserException(*exception_args)

def _pretty_filename(filename):
    a = os.path.abspath(filename)
    r = os.path.relpath(filename)
    return a if len(a) < len(r) else r

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
        raise ParserException('Cannot evaluate expression', str(ex))

def validate_search_directories(dirnames):
    dirnames = set(dirnames)
    dirnames = list(map(os.path.abspath, dirnames))
    for d1 in dirnames:
        for d2 in dirnames:
            if d1 == d2:
                continue
            _enforce(not d1.startswith(d2), 'Nested search directories are not allowed', d1, d2)
            _enforce(d1.split(os.path.sep)[-1] != d2.split(os.path.sep)[-1], 'Root namespaces must be unique', d1, d2)
    return dirnames

def validate_namespace_name(name):
    for component in name.split('.'):
        _enforce(re.match(r'[a-z][a-z0-9_]*$', component), 'Invalid namespace name', name)
    _enforce(len(name) <= MAX_FULL_TYPE_NAME_LEN, 'Namespace name is too long', name)

def validate_compound_type_full_name(name):
    _enforce('.' in name, 'Full type name must explicitly specify its namespace', name)
    short_name = name.split('.')[-1]
    namespace = '.'.join(name.split('.')[:-1])
    validate_namespace_name(namespace)
    _enforce(re.match(r'[A-Z][A-Za-z0-9_]*$', short_name), 'Invalid type name', name)
    _enforce(len(name) <= MAX_FULL_TYPE_NAME_LEN, 'Type name is too long', name)

def validate_attribute_name(name):
    _enforce(re.match(r'[a-zA-Z][a-zA-Z0-9_]*$', name), 'Invalid attribute name', name)

def tokenize_dsdl_definition(text):
    for idx, line in enumerate(text.splitlines()):
        line = re.sub('#.*', '', line).strip()  # Remove comments and leading/trailing whitespaces
        if line:
            tokens = [tk for tk in line.split() if tk]
            yield idx + 1, tokens

class Parser:
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
        raise ParserException('File was not found in search directories', filename)

    def _full_typename_and_dtid_from_filename(self, filename):
        basename = os.path.basename(filename)
        items = basename.split('.')
        if (len(items) != 2 and len(items) != 3) or items[-1] != 'uavcan':
            raise ParserException('Invalid file name', basename)
        if len(items) == 2:
            default_dtid, name = None, items[0]
        else:
            default_dtid, name = items[0], items[1]
            try:
                default_dtid = int(default_dtid)
            except ValueError:
                raise ParserException('Invalid default data type ID', default_dtid)
        full_name = self._namespace_from_filename(filename) + '.' + name
        validate_compound_type_full_name(full_name)
        return full_name, default_dtid

    def _compute_dsdl_signature(self, full_typename, fields, response_fields=None):
        text = full_typename + '\n'
        text += '\n'.join([x.get_normalized_definition() for x in fields])
        if response_fields is not None:
            text += '\n---\n'
            text += '\n'.join([x.get_normalized_definition() for x in response_fields])
        text = text.replace('\n\n', '\n')

        self.log.debug('DSDL signature of [%s] will be computed from:', full_typename)
        for ln in text.splitlines():
            self.log.debug('    %s', ln)

        return compute_signature(text)

    def _locate_compound_type_definition(self, referencing_filename, typename):
        def locate_namespace_directory(namespace):
            root_namespace, *sub_namespace_components = namespace.split('.')
            for directory in self.search_dirs:
                if directory.split(os.path.sep)[-1] == root_namespace:
                    return os.path.join(directory, *sub_namespace_components)
            raise ParserException('Unknown namespace', namespace)

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
                except Exception as ex:
                    self.log.info('Unknown file [%s], skipping... [%s]', _pretty_filename(fn), ex)
                if full_typename == fn_full_typename:
                    return fn
        raise ParserException('Type definition not found', typename)

    def _parse_array_type(self, filename, value_typedef, size_spec, cast_mode):
        self.log.debug('Parsing the array value type [%s]...', value_typedef)
        value_type = self._parse_type(filename, value_typedef, cast_mode)
        _enforce(value_type.category != value_type.CATEGORY_ARRAY, 'Multidimensional arrays are not allowed')
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
            raise ParserException('Invalid array size specifier (note: allowed [<=X], [<X], [X])', size_spec)
        _enforce(max_size > 0, 'Array size must be positive', max_size)
        return ArrayType(value_type, mode, max_size)

    def _parse_primitive_type(self, filename, base_name, bitlen, cast_mode):
        if cast_mode is None or cast_mode == 'saturated':
            cast_mode = PrimitiveType.CAST_MODE_SATURATED
        elif cast_mode == 'truncated':
            cast_mode = PrimitiveType.CAST_MODE_TRUNCATED
        else:
            raise ParserException('Invalid cast mode', cast_mode)

        if base_name == 'bool':
            return PrimitiveType(PrimitiveType.KIND_BOOLEAN, 1, cast_mode)
        try:
            kind = {
                'uint' : PrimitiveType.KIND_UNSIGNED_INT,
                'int'  : PrimitiveType.KIND_SIGNED_INT,
                'float': PrimitiveType.KIND_FLOAT,
            }[base_name]
        except KeyError:
            raise ParserException('Unknown primitive type (note: compound types must be in CamelCase)')

        _enforce(2 <= bitlen <= 64, 'Invalid bit length (note: use bool instead of uint1)', bitlen)
        return PrimitiveType(kind, bitlen, cast_mode)

    def _parse_compound_type(self, filename, typedef):
        definition_filename = self._locate_compound_type_definition(filename, typedef)
        self.log.info('Nested type [%s] is defined in [%s], parsing...', typedef, _pretty_filename(definition_filename))
        t = self.parse(definition_filename)
        if t.kind == t.KIND_SERVICE:
            raise ParserException('Service types can not be nested', t)
        self.log.info('Nested type [%s] parsed successfully', typedef)
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
            _enforce(cast_mode is None, 'Cast mode specifier is not applicable for compound types', cast_mode)
            return self._parse_compound_type(filename, typedef)

    def _parse_line(self, filename, tokens):
        cast_mode = None
        if tokens[0] == 'saturated' or tokens[0] == 'truncated':
            cast_mode, *tokens = tokens

        if len(tokens) < 2:
            raise ParserException('Invalid attribute definition', tokens)

        typename, attrname, *tokens = tokens
        validate_attribute_name(attrname)
        attrtype = self._parse_type(filename, typename, cast_mode)

        if len(tokens) > 0:
            if len(tokens) < 2 or tokens[0] != '=':
                raise ParserException('Constant assignment expected', tokens)
            if attrtype.category != Type.CATEGORY_PRIMITIVE:
                raise ParserException('Only primitive types allowed for constants', attrtype)
            expression = ' '.join(tokens[1:])
            evaluate_expression(expression)  # Validation
            return Constant(attrtype, attrname, expression)
        else:
            return Field(attrtype, attrname)

    def parse(self, filename):
        filename = os.path.abspath(filename)
        with open(filename) as f:
            text = f.read()
        try:
            full_typename, default_dtid = self._full_typename_and_dtid_from_filename(filename)
            numbered_lines = list(tokenize_dsdl_definition(text))

            all_attributes_names = set()
            fields, constants, resp_fields, resp_constants = [], [], [], []
            response_part = False
            for num, tokens in numbered_lines:
                if tokens == ['---']:
                    response_part = True
                    continue
                try:
                    attr = self._parse_line(filename, tokens)

                    if attr.name in all_attributes_names:
                        raise ParserException('Duplicated attribute name', attr.name)
                    all_attributes_names.add(attr.name)

                    if isinstance(attr, Constant):
                        (resp_constants if response_part else constants).append(attr)
                    elif isinstance(attr, Field):
                        (resp_fields if response_part else fields).append(attr)
                    else:
                        raise ParserException('Unknown attribute', attr)
                except ParserException as ex:
                    if not ex.line:
                        ex.line = num
                    raise ex
                except Exception as ex:
                    raise ParserException('Internal error', str(ex), line=num) from ex

            if response_part:
                dsdl_signature = self._compute_dsdl_signature(full_typename, fields, resp_fields)
                typedef = CompoundType(full_typename, CompoundType.KIND_SERVICE, dsdl_signature, filename, default_dtid)
                typedef.request_fields = fields
                typedef.request_constants = constants
                typedef.response_fields = resp_fields
                typedef.response_constants = resp_constants
            else:
                dsdl_signature = self._compute_dsdl_signature(full_typename, fields)
                typedef = CompoundType(full_typename, CompoundType.KIND_MESSAGE, dsdl_signature, filename, default_dtid)
                typedef.fields = fields
                typedef.constants = constants

            self.log.info('Type [%s], default DTID [%s], signature [%08x], interpretation:',
                          full_typename, default_dtid, dsdl_signature)
            for ln in typedef.get_normalized_attributes_definitions().splitlines():
                self.log.info('    %s', ln)
            return typedef
        except ParserException as ex:
            if not ex.file:
                ex.file = filename
            raise ex
        except Exception as ex:
            raise ParserException('Internal error', str(ex), file=filename) from ex


if __name__ == '__main__':
    import sys
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(levelname)s: %(message)s')

    if not sys.argv[1:]:
        self_directory = os.path.dirname(__file__)
        test_dir = os.path.join(self_directory, '..', '..', 'dsdl_test_data')
        test_dir = os.path.normpath(test_dir)
        parser = Parser([os.path.join(test_dir, 'root_a'), os.path.join(test_dir, 'root_b')])
        parser.log.setLevel(logging.DEBUG)
        t = parser.parse(os.path.join(test_dir, 'root_a', 'ns1', 'ns9', '425.BeginFirmwareUpdate.uavcan'))
    else:
        search_dirs = sys.argv[1:-1]
        filename = sys.argv[-1]
        parser = Parser(search_dirs)
        parser.log.setLevel(logging.DEBUG)
        t = parser.parse(filename)
