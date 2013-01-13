#!/usr/bin/env python

'''
fit best estimate of magnetometer offsets
'''

import sys, time, os, math

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("magfit.py [options]")
parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
parser.add_option("--minspeed", type='float', default=5.0, help="minimum ground speed to use")

(opts, args) = parser.parse_args()

if opts.mav10:
    os.environ['MAVLINK10'] = '1'
import mavutil

if len(args) < 1:
    print("Usage: magfit.py [options] <LOGFILE...>")
    sys.exit(1)

class vec3(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def __str__(self):
        return "%.1f %.1f %.1f" % (self.x, self.y, self.z)

def heading_error1(parm, data):
    from math import sin, cos, atan2, degrees
    from numpy import dot
    xofs,yofs,zofs,a1,a2,a3,a4,a5,a6,a7,a8,a9,declination = parm

    ret = []
    for d in data:
        x = d[0] + xofs
        y = d[1] + yofs
        z = d[2] + zofs
        r = d[3]
        p = d[4]
        h = d[5]

        headX = x*cos(p) + y*sin(r)*sin(p) + z*cos(r)*sin(p)
        headY = y*cos(r) - z*sin(r)
        heading = degrees(atan2(-headY,headX)) + declination
        if heading < 0:
            heading += 360
        herror = h - heading
        if herror > 180:
            herror -= 360
        if herror < -180:
            herror += 360
        ret.append(herror)
    return ret

def heading_error(parm, data):
    from math import sin, cos, atan2, degrees
    from numpy import dot
    xofs,yofs,zofs,a1,a2,a3,a4,a5,a6,a7,a8,a9,declination = parm

    a = [[1.0,a2,a3],[a4,a5,a6],[a7,a8,a9]]

    ret = []
    for d in data:
        x = d[0] + xofs
        y = d[1] + yofs
        z = d[2] + zofs
        r = d[3]
        p = d[4]
        h = d[5]
        mv = [x, y, z]
        mv2 = dot(a, mv)
        x = mv2[0]
        y = mv2[1]
        z = mv2[2]

        headX = x*cos(p) + y*sin(r)*sin(p) + z*cos(r)*sin(p)
        headY = y*cos(r) - z*sin(r)
        heading = degrees(atan2(-headY,headX)) + declination
        if heading < 0:
            heading += 360
        herror = h - heading
        if herror > 180:
            herror -= 360
        if herror < -180:
            herror += 360
        ret.append(herror)
    return ret

def fit_data(data):
    import numpy, scipy
    from scipy import optimize

    p0 = [0.0, 0.0, 0.0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0]
    p1, ier = optimize.leastsq(heading_error1, p0[:], args=(data))

#    p0 = p1[:]
#    p1, ier = optimize.leastsq(heading_error, p0[:], args=(data))

    print(p1)
    if not ier in [1, 2, 3, 4]:
        raise RuntimeError("Unable to find solution")
    return p1

def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=opts.notimestamps)

    flying = False
    gps_heading = 0.0

    data = []

    # get the current mag offsets
    m = mlog.recv_match(type='SENSOR_OFFSETS')
    offsets = vec3(m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z)

    attitude = mlog.recv_match(type='ATTITUDE')

    # now gather all the data
    while True:
        m = mlog.recv_match()
        if m is None:
            break
        if m.get_type() == "GPS_RAW":
            # flying if groundspeed more than 5 m/s
            flying = (m.v > opts.minspeed and m.fix_type == 2)
            gps_heading = m.hdg
        if m.get_type() == "ATTITUDE":
            attitude = m
        if m.get_type() == "SENSOR_OFFSETS":
            # update current offsets
            offsets = vec3(m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z)
        if not flying:
            continue
        if m.get_type() == "RAW_IMU":
            data.append((m.xmag - offsets.x, m.ymag - offsets.y, m.zmag - offsets.z, attitude.roll, attitude.pitch, gps_heading))
    print("Extracted %u data points" % len(data))
    print("Current offsets: %s" % offsets)
    ofs2 = fit_data(data)
    print("Declination estimate: %.1f" % ofs2[-1])
    new_offsets = vec3(ofs2[0], ofs2[1], ofs2[2])
    a = [[ofs2[3], ofs2[4], ofs2[5]],
         [ofs2[6], ofs2[7], ofs2[8]],
         [ofs2[9], ofs2[10], ofs2[11]]]
    print(a)
    print("New offsets    : %s" % new_offsets)

total = 0.0
for filename in args:
    magfit(filename)
