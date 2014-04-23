#!/usr/bin/env python

'''
fit best estimate of magnetometer rotation to GPS data
'''

import sys, time, os, math

from optparse import OptionParser
parser = OptionParser("magfit_rotation_gps.py [options]")
parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--declination", default=0.0, type='float', help="magnetic declination")
parser.add_option("--min-speed", default=4.0, type='float', help="minimum GPS speed")

(opts, args) = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3, Matrix3
from math import radians, degrees, sin, cos, atan2

if len(args) < 1:
    print("Usage: magfit_rotation.py [options] <LOGFILE...>")
    sys.exit(1)

class Rotation(object):
    def __init__(self, roll, pitch, yaw, r):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.r = r

def in_rotations_list(rotations, m):
    for r in rotations:
        m2 = m.transposed() * r.r
        (r, p, y) = m2.to_euler()
        if (abs(r) < radians(1) and
            abs(p) < radians(1) and
            abs(y) < radians(1)):
            return True
    return False

def generate_rotations():
    '''generate all 90 degree rotations'''
    rotations = []
    for yaw in [0, 90, 180, 270]:
        for pitch in [0, 90, 180, 270]:
            for roll in [0, 90, 180, 270]:
                m = Matrix3()
                m.from_euler(radians(roll), radians(pitch), radians(yaw))
                if not in_rotations_list(rotations, m):
                    rotations.append(Rotation(roll, pitch, yaw, m))
    return rotations

def angle_diff(angle1, angle2):
    '''give the difference between two angles in degrees'''
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret

def heading_difference(mag, attitude, declination):
    r = attitude.roll
    p = attitude.pitch
    headX = mag.x*cos(p) + mag.y*sin(r)*sin(p) + mag.z*cos(r)*sin(p)
    headY = mag.y*cos(r) - mag.z*sin(r)
    heading = degrees(atan2(-headY,headX)) + declination
    heading2 = degrees(attitude.yaw)
    return abs(angle_diff(heading, heading2))

def add_errors(mag, attitude, total_error, rotations):
    for i in range(len(rotations)):
        r = rotations[i].r
        rmag = r * mag
        total_error[i] += heading_difference(rmag, attitude, opts.declination)
        

def magfit(logfile):
    '''find best magnetometer rotation fit to a log file'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=opts.notimestamps)

    # generate 90 degree rotations
    rotations = generate_rotations()
    print("Generated %u rotations" % len(rotations))

    count = 0
    total_error = [0]*len(rotations)
    attitude = None
    gps = None

    # now gather all the data
    while True:
        m = mlog.recv_match()
        if m is None:
            break
        if m.get_type() == "ATTITUDE":
            attitude = m
        if m.get_type() == "GPS_RAW_INT":
            gps = m
        if m.get_type() == "RAW_IMU":
            mag = Vector3(m.xmag, m.ymag, m.zmag)
            if attitude is not None and gps is not None and gps.vel > opts.min_speed*100 and gps.fix_type>=3:
                add_errors(mag, attitude, total_error, rotations)
            count += 1

    best_i = 0
    best_err = total_error[0]
    for i in range(len(rotations)):
        r = rotations[i]
        print("(%u,%u,%u) err=%.2f" % (
            r.roll,
            r.pitch,
            r.yaw,
            total_error[i]/count))
        if total_error[i] < best_err:
            best_i = i
            best_err = total_error[i]
    r = rotations[best_i]
    print("Best rotation (%u,%u,%u) err=%.2f" % (
        r.roll,
        r.pitch,
        r.yaw,
        best_err/count))

for filename in args:
    magfit(filename)
