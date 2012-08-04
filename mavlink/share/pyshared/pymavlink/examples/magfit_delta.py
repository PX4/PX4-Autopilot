#!/usr/bin/env python

'''
fit best estimate of magnetometer offsets using the algorithm from
Bill Premerlani
'''

import sys, time, os, math

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

# command line option handling
from optparse import OptionParser
parser = OptionParser("magfit_delta.py [options]")
parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
parser.add_option("--verbose", action='store_true', default=False, help="verbose offset output")
parser.add_option("--gain", type='float', default=0.01, help="algorithm gain")
parser.add_option("--noise", type='float', default=0, help="noise to add")
parser.add_option("--max-change", type='float', default=10, help="max step change")
parser.add_option("--min-diff", type='float', default=50, help="min mag vector delta")
parser.add_option("--history", type='int', default=20, help="how many points to keep")
parser.add_option("--repeat", type='int', default=1, help="number of repeats through the data")

(opts, args) = parser.parse_args()

if opts.mav10:
    os.environ['MAVLINK10'] = '1'
import mavutil
from rotmat import Vector3, Matrix3

if len(args) < 1:
    print("Usage: magfit_delta.py [options] <LOGFILE...>")
    sys.exit(1)

def noise():
    '''a noise vector'''
    from random import gauss
    v = Vector3(gauss(0, 1), gauss(0, 1), gauss(0, 1))
    v.normalize()
    return v * opts.noise

def find_offsets(data, ofs):
    '''find mag offsets by applying Bills "offsets revisited" algorithm
       on the data

       This is an implementation of the algorithm from:
          http://gentlenav.googlecode.com/files/MagnetometerOffsetNullingRevisited.pdf
       '''

    # a limit on the maximum change in each step
    max_change = opts.max_change

    # the gain factor for the algorithm
    gain = opts.gain

    data2 = []
    for d in data:
        d = d.copy() + noise()
        d.x = float(int(d.x + 0.5))
        d.y = float(int(d.y + 0.5))
        d.z = float(int(d.z + 0.5))
        data2.append(d)
    data = data2

    history_idx = 0
    mag_history = data[0:opts.history]
    
    for i in range(opts.history, len(data)):
        B1 = mag_history[history_idx] + ofs
        B2 = data[i] + ofs
        
        diff = B2 - B1
        diff_length = diff.length()
        if diff_length <= opts.min_diff:
            # the mag vector hasn't changed enough - we don't get any
            # information from this
            history_idx = (history_idx+1) % opts.history
            continue

        mag_history[history_idx] = data[i]
        history_idx = (history_idx+1) % opts.history

        # equation 6 of Bills paper
        delta = diff * (gain * (B2.length() - B1.length()) / diff_length)

        # limit the change from any one reading. This is to prevent
        # single crazy readings from throwing off the offsets for a long
        # time
        delta_length = delta.length()
        if max_change != 0 and delta_length > max_change:
            delta *= max_change / delta_length

        # set the new offsets
        ofs = ofs - delta

        if opts.verbose:
            print ofs
    return ofs


def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''

    print("Processing log %s" % filename)

    # open the log file
    mlog = mavutil.mavlink_connection(filename, notimestamps=opts.notimestamps)

    data = []
    mag = None
    offsets = Vector3(0,0,0)
    
    # now gather all the data
    while True:
        # get the next MAVLink message in the log
        m = mlog.recv_match(condition=opts.condition)
        if m is None:
            break
        if m.get_type() == "SENSOR_OFFSETS":
            # update offsets that were used during this flight
            offsets = Vector3(m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z)
        if m.get_type() == "RAW_IMU" and offsets != None:
            # extract one mag vector, removing the offsets that were
            # used during that flight to get the raw sensor values
            mag = Vector3(m.xmag, m.ymag, m.zmag) - offsets
            data.append(mag)

    print("Extracted %u data points" % len(data))
    print("Current offsets: %s" % offsets)

    # run the fitting algorithm
    ofs = offsets
    ofs = Vector3(0,0,0)
    for r in range(opts.repeat):
        ofs = find_offsets(data, ofs)
        print('Loop %u offsets %s' % (r, ofs))
        sys.stdout.flush()
    print("New offsets: %s" % ofs)

total = 0.0
for filename in args:
    magfit(filename)
