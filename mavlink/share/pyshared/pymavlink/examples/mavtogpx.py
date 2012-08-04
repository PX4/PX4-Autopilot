#!/usr/bin/env python

'''
example program to extract GPS data from a mavlink log, and create a GPX
file, for loading into google earth
'''

import sys, struct, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("mavtogpx.py [options]")
parser.add_option("--condition",dest="condition", default=None, help="select packets by a condition")
parser.add_option("--nofixcheck", default=False, action='store_true', help="don't check for GPS fix")
parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
(opts, args) = parser.parse_args()

if opts.mav10:
    os.environ['MAVLINK10'] = '1'
import mavutil

if len(args) < 1:
    print("Usage: mavtogpx.py <LOGFILE>")
    sys.exit(1)

def mav_to_gpx(infilename, outfilename):
    '''convert a mavlink log file to a GPX file'''

    mlog = mavutil.mavlink_connection(infilename)
    outf = open(outfilename, mode='w')

    def process_packet(m):
        t = time.localtime(m._timestamp)
        outf.write('''<trkpt lat="%s" lon="%s">
  <ele>%s</ele>
  <time>%s</time>
  <course>%s</course>
  <speed>%s</speed>
  <fix>3d</fix>
</trkpt>
''' % (m.lat, m.lon, m.alt,
       time.strftime("%Y-%m-%dT%H:%M:%SZ", t),
       m.hdg, m.v))

    def add_header():
        outf.write('''<?xml version="1.0" encoding="UTF-8"?>
<gpx
  version="1.0"
  creator="pymavlink"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xmlns="http://www.topografix.com/GPX/1/0"
  xsi:schemaLocation="http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd">
<trk>
<trkseg>
''')

    def add_footer():
        outf.write('''</trkseg>
</trk>
</gpx>
''')

    add_header()       

    count=0
    while True:
        m = mlog.recv_match(type='GPS_RAW', condition=opts.condition)
        if m is None: break
        if m.fix_type != 2 and not opts.nofixcheck:
            continue
        if m.lat == 0.0 or m.lon == 0.0:
            continue
        process_packet(m)
        count += 1
    add_footer()
    print("Created %s with %u points" % (outfilename, count))
    

for infilename in args:
    outfilename = infilename + '.gpx'
    mav_to_gpx(infilename, outfilename)
