#!/usr/bin/env python
'''
mavlink python parse functions

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import xml.parsers.expat, os, errno, time, sys, operator, mavutil

PROTOCOL_0_9 = "0.9"
PROTOCOL_1_0 = "1.0"

class MAVParseError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()
    def __str__(self):
        return self.message

class MAVField(object):
    def __init__(self, name, type, print_format, xml, description=''):
        self.name = name
        self.name_upper = name.upper()
        self.description = description
        self.array_length = 0
        self.omit_arg = False
        self.const_value = None
        self.print_format = print_format
        lengths = {
        'float'    : 4,
        'double'   : 8,
        'char'     : 1,
        'int8_t'   : 1,
        'uint8_t'  : 1,
        'uint8_t_mavlink_version'  : 1,
        'int16_t'  : 2,
        'uint16_t' : 2,
        'int32_t'  : 4,
        'uint32_t' : 4,
        'int64_t'  : 8,
        'uint64_t' : 8,
        }

        if type=='uint8_t_mavlink_version':
            type = 'uint8_t'
            self.omit_arg = True
            self.const_value = xml.version

        aidx = type.find("[")
        if aidx != -1:
            assert type[-1:] == ']'
            self.array_length = int(type[aidx+1:-1])
            type = type[0:aidx]
            if type == 'array':
                type = 'int8_t'
        if type in lengths:
            self.type_length = lengths[type]
            self.type = type
        elif (type+"_t") in lengths:
            self.type_length = lengths[type+"_t"]
            self.type = type+'_t'
        else:
            raise MAVParseError("unknown type '%s'" % type)
        if self.array_length != 0:
            self.wire_length = self.array_length * self.type_length
        else:
            self.wire_length = self.type_length
        self.type_upper = self.type.upper()

    def gen_test_value(self, i):
        '''generate a testsuite value for a MAVField'''
        if self.const_value:
            return self.const_value
        elif self.type == 'float':
            return 17.0 + self.wire_offset*7 + i
        elif self.type == 'double':
            return 123.0 + self.wire_offset*7 + i
        elif self.type == 'char':
            return chr(ord('A') + (self.wire_offset + i)%26)
        elif self.type in [ 'int8_t', 'uint8_t' ]:
            return (5 + self.wire_offset*67 + i) & 0xFF
        elif self.type in ['int16_t', 'uint16_t']:
            return (17235 + self.wire_offset*52 + i) & 0xFFFF
        elif self.type in ['int32_t', 'uint32_t']:
            return (963497464 + self.wire_offset*52 + i)&0xFFFFFFFF
        elif self.type in ['int64_t', 'uint64_t']:
            return 93372036854775807 + self.wire_offset*63 + i
        else:
            raise MAVError('unknown type %s' % self.type)

    def set_test_value(self):
        '''set a testsuite value for a MAVField'''
        if self.array_length:
            self.test_value = []
            for i in range(self.array_length):
                self.test_value.append(self.gen_test_value(i))
        else:
                self.test_value = self.gen_test_value(0)
        if self.type == 'char' and self.array_length:
            v = ""
            for c in self.test_value:
                v += c
            self.test_value = v[:-1]


class MAVType(object):
    def __init__(self, name, id, linenumber, description=''):
        self.name = name
        self.name_lower = name.lower()
        self.linenumber = linenumber
        self.id = int(id)
        self.description = description
        self.fields = []
        self.fieldnames = []

class MAVEnumParam(object):
    def __init__(self, index, description=''):
        self.index = index
        self.description = description

class MAVEnumEntry(object):
    def __init__(self, name, value, description='', end_marker=False):
        self.name = name
        self.value = value
        self.description = description
        self.param = []
        self.end_marker = end_marker

class MAVEnum(object):
    def __init__(self, name, linenumber, description=''):
        self.name = name
        self.description = description
        self.entry = []
        self.highest_value = 0
        self.linenumber = linenumber

class MAVXML(object):
    '''parse a mavlink XML file'''
    def __init__(self, filename, wire_protocol_version=PROTOCOL_0_9):
        self.filename = filename
        self.basename = os.path.basename(filename)
        if self.basename.lower().endswith(".xml"):
            self.basename = self.basename[:-4]
        self.basename_upper = self.basename.upper()
        self.message = []
        self.enum = []
        self.parse_time = time.asctime()
        self.version = 2
        self.include = []
        self.wire_protocol_version = wire_protocol_version

        if wire_protocol_version == PROTOCOL_0_9:
            self.protocol_marker = ord('U')
            self.sort_fields = False
            self.little_endian = False
            self.crc_extra = False
        elif wire_protocol_version == PROTOCOL_1_0:
            self.protocol_marker = 0xFE
            self.sort_fields = True
            self.little_endian = True
            self.crc_extra = True
        else:
            print("Unknown wire protocol version")
            print("Available versions are: %s %s" % (PROTOCOL_0_9, PROTOCOL_1_0))
            raise MAVParseError('Unknown MAVLink wire protocol version %s' % wire_protocol_version)

        in_element_list = []

        def check_attrs(attrs, check, where):
            for c in check:
                if not c in attrs:
                    raise MAVParseError('expected missing %s "%s" attribute at %s:%u' % (
                        where, c, filename, p.CurrentLineNumber))

        def start_element(name, attrs):
            in_element_list.append(name)
            in_element = '.'.join(in_element_list)
            #print in_element
            if in_element == "mavlink.messages.message":
                check_attrs(attrs, ['name', 'id'], 'message')
                self.message.append(MAVType(attrs['name'], attrs['id'], p.CurrentLineNumber))
            elif in_element == "mavlink.messages.message.field":
                check_attrs(attrs, ['name', 'type'], 'field')
                if 'print_format' in attrs:
                    print_format = attrs['print_format']
                else:
                    print_format = None
                self.message[-1].fields.append(MAVField(attrs['name'], attrs['type'],
                                                        print_format, self))
            elif in_element == "mavlink.enums.enum":
                check_attrs(attrs, ['name'], 'enum')
                self.enum.append(MAVEnum(attrs['name'], p.CurrentLineNumber))
            elif in_element == "mavlink.enums.enum.entry":
                check_attrs(attrs, ['name'], 'enum entry')
                if 'value' in attrs:
                    value = int(attrs['value'])
                else:
                    value = self.enum[-1].highest_value + 1
                if (value > self.enum[-1].highest_value):
                    self.enum[-1].highest_value = value
                self.enum[-1].entry.append(MAVEnumEntry(attrs['name'], value))
            elif in_element == "mavlink.enums.enum.entry.param":
                check_attrs(attrs, ['index'], 'enum param')
                self.enum[-1].entry[-1].param.append(MAVEnumParam(attrs['index']))

        def end_element(name):
            in_element = '.'.join(in_element_list)
            if in_element == "mavlink.enums.enum":
                # add a ENUM_END
                self.enum[-1].entry.append(MAVEnumEntry("%s_ENUM_END" % self.enum[-1].name,
                                                        self.enum[-1].highest_value+1, end_marker=True))
            in_element_list.pop()

        def char_data(data):
            in_element = '.'.join(in_element_list)
            if in_element == "mavlink.messages.message.description":
                self.message[-1].description += data
            elif in_element == "mavlink.messages.message.field":
                self.message[-1].fields[-1].description += data
            elif in_element == "mavlink.enums.enum.description":
                self.enum[-1].description += data
            elif in_element == "mavlink.enums.enum.entry.description":
                self.enum[-1].entry[-1].description += data
            elif in_element == "mavlink.enums.enum.entry.param":
                self.enum[-1].entry[-1].param[-1].description += data
            elif in_element == "mavlink.version":
                self.version = int(data)
            elif in_element == "mavlink.include":
                self.include.append(data)

        f = open(filename, mode='rb')
        p = xml.parsers.expat.ParserCreate()
        p.StartElementHandler = start_element
        p.EndElementHandler = end_element
        p.CharacterDataHandler = char_data
        p.ParseFile(f)
        f.close()

        self.message_lengths = [ 0 ] * 256
        self.message_crcs = [ 0 ] * 256
        self.message_names = [ None ] * 256
        self.largest_payload = 0

        for m in self.message:
            m.wire_length = 0
            m.fieldnames = []
            m.ordered_fieldnames = []
            if self.sort_fields:
                m.ordered_fields = sorted(m.fields,
                                          key=operator.attrgetter('type_length'),
                                          reverse=True)
            else:
                m.ordered_fields = m.fields
            for f in m.fields:
                m.fieldnames.append(f.name)
            for f in m.ordered_fields:
                f.wire_offset = m.wire_length
                m.wire_length += f.wire_length
                m.ordered_fieldnames.append(f.name)
                f.set_test_value()
            m.num_fields = len(m.fieldnames)
            if m.num_fields > 64:
                raise MAVParseError("num_fields=%u : Maximum number of field names allowed is" % (
                    m.num_fields, 64))
            m.crc_extra = message_checksum(m)
            self.message_lengths[m.id] = m.wire_length
            self.message_names[m.id] = m.name
            self.message_crcs[m.id] = m.crc_extra
            if m.wire_length > self.largest_payload:
                self.largest_payload = m.wire_length

            if m.wire_length+8 > 64:
                print("Note: message %s is longer than 64 bytes long (%u bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit." % (m.name, m.wire_length+8))

    def __str__(self):
        return "MAVXML for %s from %s (%u message, %u enums)" % (
            self.basename, self.filename, len(self.message), len(self.enum))
    

def message_checksum(msg):
    '''calculate a 8-bit checksum of the key fields of a message, so we
       can detect incompatible XML changes'''
    crc = mavutil.x25crc(msg.name + ' ')
    for f in msg.ordered_fields:
        crc.accumulate(f.type + ' ')
        crc.accumulate(f.name + ' ')
        if f.array_length:
            crc.accumulate(chr(f.array_length))
    return (crc.crc&0xFF) ^ (crc.crc>>8)

def merge_enums(xml):
    '''merge enums between XML files'''
    emap = {}
    for x in xml:
        newenums = []
        for enum in x.enum:
            if enum.name in emap:
                emap[enum.name].entry.pop() # remove end marker
                emap[enum.name].entry.extend(enum.entry)
                print("Merged enum %s" % enum.name)
            else:
                newenums.append(enum)
                emap[enum.name] = enum
        x.enum = newenums
    # sort by value
    for e in emap:
        emap[e].entry = sorted(emap[e].entry,
                               key=operator.attrgetter('value'),
                               reverse=False)


def check_duplicates(xml):
    '''check for duplicate message IDs'''

    merge_enums(xml)

    msgmap = {}
    enummap = {}
    for x in xml:
        for m in x.message:
            if m.id in msgmap:
                print("ERROR: Duplicate message id %u for %s (%s:%u) also used by %s" % (
                    m.id, m.name,
                    x.filename, m.linenumber,
                    msgmap[m.id]))
                return True
            fieldset = set()
            for f in m.fields:
                if f.name in fieldset:
                    print("ERROR: Duplicate field %s in message %s (%s:%u)" % (
                        f.name, m.name,
                        x.filename, m.linenumber))
                    return True
                fieldset.add(f.name)
            msgmap[m.id] = '%s (%s:%u)' % (m.name, x.filename, m.linenumber)
        for enum in x.enum:
            for entry in enum.entry:
                s1 = "%s.%s" % (enum.name, entry.name)
                s2 = "%s.%s" % (enum.name, entry.value)
                if s1 in enummap or s2 in enummap:
                    print("ERROR: Duplicate enums %s/%s at %s:%u and %s" % (
                        s1, entry.value, x.filename, enum.linenumber,
                        enummap.get(s1) or enummap.get(s2)))
                    return True
                enummap[s1] = "%s:%u" % (x.filename, enum.linenumber)
                enummap[s2] = "%s:%u" % (x.filename, enum.linenumber)
                    
    return False



def total_msgs(xml):
    '''count total number of msgs'''
    count = 0
    for x in xml:
        count += len(x.message)
    return count

def mkdir_p(dir):
    try:
        os.makedirs(dir)
    except OSError as exc:
        if exc.errno == errno.EEXIST:
            pass
        else: raise

# check version consistent
# add test.xml
# finish test suite
# printf style error macro, if defined call errors
