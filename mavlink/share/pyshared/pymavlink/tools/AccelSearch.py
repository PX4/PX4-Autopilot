#!/usr/bin/env python

'''
search a set of log files for bad accel values
'''

import sys, time, os, glob
import zipfile

from pymavlink import mavutil

# extra imports for pyinstaller
import json
from pymavlink.dialects.v10 import ardupilotmega

search_dirs = ['c:\Program Files\APM Planner', 
               'c:\Program Files\Mission Planner', 
               'c:\Program Files (x86)\APM Planner',
               'c:\Program Files (x86)\Mission Planner']
results = 'SearchResults.zip'
email = 'Craig Elder <craig@3drobotics.com>'

from optparse import OptionParser
parser = OptionParser("AccelSearch.py [options]")
parser.add_option("--directory", action='append', default=search_dirs, help="directories to search")
parser.add_option("--post-boot", action='store_true', help="post boot only")
parser.add_option("--init-only", action='store_true', help="init only")
parser.add_option("--single-axis", action='store_true', help="single axis only")

(opts, args) = parser.parse_args()

logcount = 0

def AccelSearch(filename):
    global logcount
    mlog = mavutil.mavlink_connection(filename)
    badcount = 0
    badval = None
    have_ok = False
    last_t = 0
    while True:
        m = mlog.recv_match(type=['PARAM_VALUE','RAW_IMU'])
        if m is None:
            if last_t != 0:
                logcount += 1
            return False
        if m.get_type() == 'PARAM_VALUE':
            if m.param_id.startswith('INS_PRODUCT_ID'):
                if m.param_value not in [0.0, 5.0]:
                    return False
        if m.get_type() == 'RAW_IMU':
            if m.time_usec < last_t:
                have_ok = False
            last_t = m.time_usec
            if abs(m.xacc) >= 3000 and abs(m.yacc) > 3000 and abs(m.zacc) > 3000 and not opts.single_axis:
                if opts.post_boot and not have_ok:
                    continue
                if opts.init_only and have_ok:
                    continue
                print have_ok, last_t, m
                break
            # also look for a single axis that stays nearly constant at a large value
            for axes in ['xacc', 'yacc', 'zacc']:
                value1 = getattr(m, axes)
                if abs(value1) > 2000:
                    if badval is None:
                        badcount = 1
                        badval = m
                        continue
                    value2 = getattr(badval, axes)
                    if abs(value1 - value2) < 30:
                        badcount += 1
                        badval = m
                        if badcount > 5:
                            logcount += 1
                            if opts.init_only and have_ok:
                                continue
                            print have_ok, badcount, badval, m
                            return True
                    else:
                        badcount = 1
                        badval = m
            if badcount == 0:
                have_ok = True
    if last_t != 0:
        logcount += 1
    return True
        
found = []
directories = opts.directory

# allow drag and drop
if len(sys.argv) > 1:
    directories = sys.argv[1:]

filelist = []

for d in directories:
    if not os.path.exists(d):
        continue
    if os.path.isdir(d):
        print("Searching in %s" % d)
        for (root, dirs, files) in os.walk(d):
            for f in files:
                if not f.endswith('.tlog'):
                    continue
                path = os.path.join(root, f)
                filelist.append(path)
    elif d.endswith('.tlog'):
        filelist.append(d)

for i in range(len(filelist)):
    f = filelist[i]
    print("Checking %s ... [found=%u logcount=%u i=%u/%u]" % (f, len(found), logcount, i, len(filelist)))
    if AccelSearch(f):
        found.append(f)
        

if len(found) == 0:
    print("No matching files found - all OK!")
    raw_input('Press enter to close')
    sys.exit(0)

print("Creating zip file %s" % results)
try:
    zip = zipfile.ZipFile(results, 'w')
except Exception:
    print("Unable to create zip file %s" % results)
    print("Please send matching files manually")
    for f in found:
        print('MATCHED: %s' % f)
    raw_input('Press enter to close')
    sys.exit(1)

for f in found:
    zip.write(f, arcname=os.path.basename(f))
zip.close()

print('==============================================')
print("Created %s with %u of %u matching logs" % (results, len(found), logcount))
print("Please send this file to %s" % email)
print('==============================================')

raw_input('Press enter to close')
sys.exit(0)
