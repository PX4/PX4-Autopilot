#!/usr/bin/env python

'''
convert a MAVLink tlog file to a MATLab mfile
'''

import sys, os
import re
from pymavlink import mavutil

def process_tlog(filename):
    '''convert a tlog to a .m file'''

    print("Processing %s" % filename)
    
    mlog = mavutil.mavlink_connection(filename, dialect=opts.dialect, zero_time_base=True)
    
    # first walk the entire file, grabbing all messages into a hash of lists,
    #and the first message of each type into a hash
    msg_types = {}
    msg_lists = {}

    types = opts.types
    if types is not None:
        types = types.split(',')

    # note that Octave doesn't like any extra '.', '*', '-', characters in the filename
    (head, tail) = os.path.split(filename)
    basename = '.'.join(tail.split('.')[:-1])
    mfilename = re.sub('[\.\-\+\*]','_', basename) + '.m'
    # Octave also doesn't like files that don't start with a letter
    if (re.match('^[a-zA-z]', mfilename) == None):
        mfilename = 'm_' + mfilename

    if head is not None:
        mfilename = os.path.join(head, mfilename)
    print("Creating %s" % mfilename)

    f = open(mfilename, "w")

    type_counters = {}

    while True:
        m = mlog.recv_match(condition=opts.condition)
        if m is None:
            break

        if types is not None and m.get_type() not in types:
            continue
        if m.get_type() == 'BAD_DATA':
            continue
        
        fieldnames = m._fieldnames
        mtype = m.get_type()
        if mtype in ['FMT', 'PARM']:
            continue

        if mtype not in type_counters:
            type_counters[mtype] = 0
            f.write("%s.heading = {'timestamp'" % mtype)
            for field in fieldnames:
                val = getattr(m, field)
                if not isinstance(val, str):
                    f.write(",'%s'" % field)
            f.write("};\n")

        type_counters[mtype] += 1
        f.write("%s.data(%u,:) = [%f" % (mtype, type_counters[mtype], m._timestamp))
        for field in m._fieldnames:
            val = getattr(m, field)
            if not isinstance(val, str):
                f.write(",%f" % val)
        f.write("];\n")
    f.close()

from optparse import OptionParser
parser = OptionParser("mavtomfile.py [options]")

parser.add_option("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_option("-o", "--output", default=None, help="output filename")
parser.add_option("--types",  default=None, help="types of messages (comma separated)")
parser.add_option("--dialect",  default="ardupilotmega", help="MAVLink dialect")
(opts, args) = parser.parse_args()

if len(args) < 1:
    print("Usage: mavtomfile.py [options] <LOGFILE>")
    sys.exit(1)

for filename in args:
    process_tlog(filename)
