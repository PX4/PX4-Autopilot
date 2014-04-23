#!/usr/bin/env python
'''
APM DataFlash log file reader

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

Partly based on SDLog2Parser by Anton Babushkin
'''

import struct, time, os
from pymavlink import mavutil

FORMAT_TO_STRUCT = {
    "b": ("b", None, int),
    "B": ("B", None, int),
    "h": ("h", None, int),
    "H": ("H", None, int),
    "i": ("i", None, int),
    "I": ("I", None, int),
    "f": ("f", None, float),
    "n": ("4s", None, str),
    "N": ("16s", None, str),
    "Z": ("64s", None, str),
    "c": ("h", 0.01, float),
    "C": ("H", 0.01, float),
    "e": ("i", 0.01, float),
    "E": ("I", 0.01, float),
    "L": ("i", 1.0e-7, float),
    "M": ("b", None, int),
    "q": ("q", None, int),
    "Q": ("Q", None, int),
    }

class DFFormat(object):
    def __init__(self, type, name, len, format, columns):
        self.type = type
        self.name = name
        self.len = len
        self.format = format
        self.columns = columns.split(',')

        msg_struct = "<"
        msg_mults = []
        msg_types = []
        for c in format:
            if ord(c) == 0:
                break
            try:
                (s, mul, type) = FORMAT_TO_STRUCT[c]
                msg_struct += s
                msg_mults.append(mul)
                msg_types.append(type)
            except KeyError as e:
                raise Exception("Unsupported format char: '%s' in message %s" % (c, name))

        self.msg_struct = msg_struct
        self.msg_types = msg_types
        self.msg_mults = msg_mults

def null_term(str):
    '''null terminate a string'''
    idx = str.find("\0")
    if idx != -1:
        str = str[:idx]
    return str

class DFMessage(object):
    def __init__(self, fmt, elements, apply_multiplier):
        self._d = {}
        self.fmt = fmt
        for i in range(len(fmt.columns)):
            mul = fmt.msg_mults[i]
            name = fmt.columns[i]
            self._d[name] = elements[i]
            if fmt.format[i] != 'M' or apply_multiplier:
                self._d[name] = fmt.msg_types[i](self._d[name])
            if fmt.msg_types[i] == str:
                self._d[name] = self._d[name]
                self._d[name] = null_term(self._d[name])
            if mul is not None and apply_multiplier:
                self._d[name] = self._d[name] * mul
        self._fieldnames = fmt.columns
        self.__dict__.update(self._d)

    def get_type(self):
        return self.fmt.name

    def __str__(self):
        ret = "%s {" % self.fmt.name
        for c in self.fmt.columns:
            ret += "%s : %s, " % (c, self._d[c])
        ret = ret[:-2] + "}"
        return ret

    def get_msgbuf(self):
        '''create a binary message buffer for a message'''
        values = []
        for i in range(len(self.fmt.columns)):
            mul = self.fmt.msg_mults[i]
            name = self.fmt.columns[i]
            if name == 'Mode' and 'ModeNum' in self.fmt.columns:
                name = 'ModeNum'
            v = self._d[name]
            if mul is not None:
                v /= mul
            values.append(v)
        return struct.pack("BBB", 0xA3, 0x95, self.fmt.type) + struct.pack(self.fmt.msg_struct, *values)
                

class DFReader(object):
    '''parse a generic dataflash file'''
    def __init__(self):
        # read the whole file into memory for simplicity
        self.msg_rate = {}
        self.new_timestamps = False
        self.interpolated_timestamps = False
        self.px4_timestamps = False
        self.px4_timebase = 0
        self.timestamp = 0
        self.verbose = False
        self.params = {}
        
    def _rewind(self):
        '''reset counters on rewind'''
        self.counts = {}
        self.counts_since_gps = {}
        self.messages = { 'MAV' : self }
        self.flightmode = "UNKNOWN"
        self.percent = 0

    def _gpsTimeToTime(self, week, sec):
        '''convert GPS week and TOW to a time in seconds since 1970'''
        epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2)
        return epoch + 86400*7*week + sec - 15

    def _find_time_base_new(self, gps):
        '''work out time basis for the log - new style'''
        t = self._gpsTimeToTime(gps.Week, gps.TimeMS*0.001)
        self.timebase = t - gps.T*0.001
        self.new_timestamps = True

    def _find_time_base_px4(self, gps):
        '''work out time basis for the log - PX4 native'''
        t = gps.GPSTime * 1.0e-6
        self.timebase = t - self.px4_timebase
        self.px4_timestamps = True

    def _find_time_base(self):
        '''work out time basis for the log'''
        self.timebase = 0
        if self._zero_time_base:
            return
        gps1 = self.recv_match(type='GPS', condition='getattr(GPS,"Week",0)!=0 or getattr(GPS,"GPSTime",0)!=0')
        if gps1 is None:
            self._rewind()
            return

        if 'GPSTime' in gps1._fieldnames:
            self._find_time_base_px4(gps1)
            self._rewind()
            return
            
        if 'T' in gps1._fieldnames:
            # it is a new style flash log with full timestamps
            self._find_time_base_new(gps1)
            self._rewind()
            return
        
        counts1 = self.counts.copy()
        gps2 = self.recv_match(type='GPS', condition='GPS.Week!=0')
        counts2 = self.counts.copy()

        if gps1 is None or gps2 is None:
            self._rewind()
            return
        
        t1 = self._gpsTimeToTime(gps1.Week, gps1.TimeMS*0.001)
        t2 = self._gpsTimeToTime(gps2.Week, gps2.TimeMS*0.001)
        if t2 == t1:
            self._rewind()
            return
        for type in counts2:
            self.msg_rate[type] = (counts2[type] - counts1[type]) / float(t2-t1)
            if self.msg_rate[type] == 0:
                self.msg_rate[type] = 1
        self._rewind()
        
    def _adjust_time_base(self, m):
        '''adjust time base from GPS message'''
        if self._zero_time_base:
            return
        if self.new_timestamps and not self.interpolated_timestamps:
            return
        if self.px4_timestamps:
            return
        if getattr(m, 'Week', None) is None:
            return
        t = self._gpsTimeToTime(m.Week, m.TimeMS*0.001)
        deltat = t - self.timebase
        if deltat <= 0:
            return
        for type in self.counts_since_gps:
            rate = self.counts_since_gps[type] / deltat
            if rate > self.msg_rate.get(type, 0):
                self.msg_rate[type] = rate
        self.msg_rate['IMU'] = 50.0
        self.msg_rate['ATT'] = 50.0
        self.timebase = t
        self.counts_since_gps = {}        

    def _set_time(self, m):
        '''set time for a message'''
        if self.px4_timestamps:
            m._timestamp = self.timebase + self.px4_timebase
        elif self._zero_time_base or (self.new_timestamps and not self.interpolated_timestamps):
            if m.get_type() in ['ATT'] and not 'TimeMS' in m._fieldnames:
                # old copter logs without TimeMS on key messages
                self.interpolated_timestamps = True
            if m.get_type() in ['GPS','GPS2']:
                m._timestamp = self.timebase + m.T*0.001
            elif 'TimeMS' in m._fieldnames:
                m._timestamp = self.timebase + m.TimeMS*0.001
            else:
                m._timestamp = self.timestamp
        else:
            rate = self.msg_rate.get(m.fmt.name, 50.0)
            count = self.counts_since_gps.get(m.fmt.name, 0)
            m._timestamp = self.timebase + count/rate
        self.timestamp = m._timestamp

    def recv_msg(self):
        return self._parse_next()

    def _add_msg(self, m):
        '''add a new message'''
        type = m.get_type()
        self.messages[type] = m
        if not type in self.counts:
            self.counts[type] = 0
        else:
            self.counts[type] += 1
        if not type in self.counts_since_gps:
            self.counts_since_gps[type] = 0
        else:
            self.counts_since_gps[type] += 1

        if type == 'TIME' and 'StartTime' in m._fieldnames:
            self.px4_timebase = m.StartTime * 1.0e-6
            self.px4_timestamps = True
        if type == 'GPS':
            self._adjust_time_base(m)
        if type == 'MODE':
            if isinstance(m.Mode, str):
                self.flightmode = m.Mode.upper()
            elif 'ModeNum' in m._fieldnames:
                self.flightmode = mavutil.mode_string_apm(m.ModeNum)
            else:
                self.flightmode = mavutil.mode_string_acm(m.Mode)
        if type == 'PARM' and getattr(m, 'Name', None) is not None:
            self.params[m.Name] = m.Value
        self._set_time(m)

    def recv_match(self, condition=None, type=None, blocking=False):
        '''recv the next message that matches the given condition
        type can be a string or a list of strings'''
        if type is not None and not isinstance(type, list):
            type = [type]
        while True:
            m = self.recv_msg()
            if m is None:
                return None
            if type is not None and not m.get_type() in type:
                continue
            if not mavutil.evaluate_condition(condition, self.messages):
                continue
            return m

    def check_condition(self, condition):
        '''check if a condition is true'''
        return mavutil.evaluate_condition(condition, self.messages)

class DFReader_binary(DFReader):
    '''parse a binary dataflash file'''
    def __init__(self, filename, zero_time_base):
        DFReader.__init__(self)
        # read the whole file into memory for simplicity
        f = open(filename, mode='rb')
        self.data = f.read()
        f.close()
        self.HEAD1 = 0xA3
        self.HEAD2 = 0x95
        self.formats = {
            0x80 : DFFormat(0x80, 'FMT', 89, 'BBnNZ', "Type,Length,Name,Format,Columns")
        }
        self._rewind()
        self._zero_time_base = zero_time_base
        self._find_time_base()
        self._rewind()

    def _rewind(self):
        '''rewind to start of log'''
        DFReader._rewind(self)
        self.offset = 0
        self.remaining = len(self.data)

    def _parse_next(self):
        '''read one message, returning it as an object'''
        if len(self.data) - self.offset < 3:
            return None
            
        hdr = self.data[self.offset:self.offset+3]
        skip_bytes = 0
        skip_type = None
        # skip over bad messages
        while (ord(hdr[0]) != self.HEAD1 or ord(hdr[1]) != self.HEAD2 or ord(hdr[2]) not in self.formats):
            if skip_type is None:
                skip_type = (ord(hdr[0]), ord(hdr[1]), ord(hdr[2]))
            skip_bytes += 1
            self.offset += 1
            if len(self.data) - self.offset < 3:
                return None
            hdr = self.data[self.offset:self.offset+3]
        msg_type = ord(hdr[2])
        if skip_bytes != 0:
            print("Skipped %u bad bytes in log %s" % (skip_bytes, skip_type))

        self.offset += 3
        self.remaining -= 3

        if not msg_type in self.formats:
            if self.verbose:
                print("unknown message type %02x" % msg_type)
            raise Exception("Unknown message type %02x" % msg_type)
        fmt = self.formats[msg_type]
        if self.remaining < fmt.len-3:
            # out of data - can often happen half way through a message
            if self.verbose:
                print("out of data")
            return None
        body = self.data[self.offset:self.offset+(fmt.len-3)]
        try:
            elements = list(struct.unpack(fmt.msg_struct, body))
        except Exception:
            print("Failed to parse %s/%s with len %u (remaining %u)" % (fmt.name, fmt.msg_struct, len(body), self.remaining))
            raise
        name = null_term(fmt.name)
        if name == 'FMT' and elements[0] not in self.formats:
            # add to formats
            # name, len, format, headings
            self.formats[elements[0]] = DFFormat(elements[0],
                                                 null_term(elements[2]), elements[1],
                                                 null_term(elements[3]), null_term(elements[4]))

        self.offset += fmt.len-3
        self.remaining -= fmt.len-3
        m = DFMessage(fmt, elements, True)
        self._add_msg(m)

        self.percent = 100.0 * (self.offset / float(len(self.data)))
        
        return m

def DFReader_is_text_log(filename):
    '''return True if a file appears to be a valid text log'''
    f = open(filename)
    ret = (f.read(8000).find('FMT, ') != -1)
    f.close()
    return ret

class DFReader_text(DFReader):
    '''parse a text dataflash file'''
    def __init__(self, filename, zero_time_base=False):
        DFReader.__init__(self)
        # read the whole file into memory for simplicity
        f = open(filename, mode='r')
        self.lines = f.readlines()
        f.close()
        self.formats = {
            'FMT' : DFFormat(0x80, 'FMT', 89, 'BBnNZ', "Type,Length,Name,Format,Columns")
        }
        self._rewind()
        self._zero_time_base = zero_time_base
        self._find_time_base()
        self._rewind()

    def _rewind(self):
        '''rewind to start of log'''
        DFReader._rewind(self)
        self.line = 0
        # find the first valid line
        while self.line < len(self.lines):
            if self.lines[self.line].startswith("FMT, "):
                break
            self.line += 1

    def _parse_next(self):
        '''read one message, returning it as an object'''
        while self.line < len(self.lines):
            s = self.lines[self.line].rstrip()
            elements = s.split(", ")
            # move to next line
            self.line += 1
            if len(elements) >= 2:
                break

        # cope with empty structures
        if len(elements) == 5 and elements[-1] == ',':
            elements[-1] = ''
            elements.append('')

        self.percent = 100.0 * (self.line / float(len(self.lines)))

        if self.line >= len(self.lines):
            return None

        msg_type = elements[0]

        if not msg_type in self.formats:
            return None
        
        fmt = self.formats[msg_type]

        if len(elements) < len(fmt.format)+1:
            # not enough columns
            return None

        elements = elements[1:]
        
        name = fmt.name.rstrip('\0')
        if name == 'FMT':
            # add to formats
            # name, len, format, headings
            self.formats[elements[2]] = DFFormat(int(elements[0]), elements[2], int(elements[1]), elements[3], elements[4])

        m = DFMessage(fmt, elements, False)
        self._add_msg(m)

        return m

if __name__ == "__main__":
    import sys
    filename = sys.argv[1]
    if filename.endswith('.log'):
        log = DFReader_text(filename)
    else:
        log = DFReader_binary(filename)
    while True:
        m = log.recv_msg()
        if m is None:
            break
        print m
