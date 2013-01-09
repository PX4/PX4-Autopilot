#!/usr/bin/env python

'''
classes for sensor models
'''

import struct, time, numpy
from math import sin, cos

from constants import *

# TODO
class Pressure(object):

    def __init__(self, time, press_abs, press_diff1, press_diff2, temperature):
        self.time = time
        self.press_abs = press_abs
        self.press_diff1 = press_diff1
        self.press_diff2 = press_diff2
        self.temperature = temperature

    def send_to_mav(self, mav):
        bar2mbar = 1.0e3
        try:
            mav.raw_pressure_send(self.time*sec2msec,
                             self.press_abs*bar2mbar, self.press_diff1*bar2mbar,
                             self.press_diff2*bar2mbar, self.temperature*100)
        except struct.error as e:
            print 'mav raw pressure packet data exceeds int bounds'

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0)

    @classmethod
    def from_state(cls, state):
        ground_press = 1.01325 #bar
        ground_tempC = 21.0
        tempC = 21.0  # TODO temp variation
        tempAvgK = T0 + (tempC + ground_tempC)/2
        pressBar = ground_press/math.exp(state.alt*(g/R)/tempAvgK)
        press_diff1 = 0 # TODO, for velocity
        press_diff2 = 0 # TODO, ?

        # TODO INSERT NOISE HERE

        return cls(time=time.time(), press_abs = pressBar, press_diff1 = press_diff1,
                   press_diff2 = press_diff2, temperature = tempC)

class Imu(object):

    def __init__(self, time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag):
        self.time = time
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc
        self.xgyro = xgyro
        self.ygyro = ygyro
        self.zgyro = zgyro
        self.xmag = xmag
        self.ymag = ymag
        self.zmag = zmag

    def send_to_mav(self, mav):
        try:
            mav.raw_imu_send(self.time*sec2msec,
                             self.xacc*mpss2mg, self.yacc*mpss2mg, self.zacc*mpss2mg,
                             self.xgyro*rad2mrad, self.ygyro*rad2mrad, self.zgyro*rad2mrad,
                             self.xmag*ga2mga, self.ymag*ga2mga, self.zmag*ga2mga)
        except struct.error as e:
            print 'mav raw imu packet data exceeds int bounds'

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)

    @classmethod
    def from_state(cls, state):

        # accelerometer
        xacc = state.xacc
        yacc = state.yacc
        zacc = state.zacc
    
        # gyroscope
        xgyro = state.p
        ygyro = state.q
        zgyro = state.r

        # mag field properties
        # setting to constants, should
        # depend on position
        magFieldStrength = 0.5
        dip = 60.0*deg2rad
        dec = 0.0*deg2rad

        magVectN = magFieldStrength*numpy.matrix([
            [cos(dip)*cos(dec)],
            [cos(dip)*sin(dec)],
            [sin(dip)]])
        magVectB = numpy.transpose(state.C_nb)*magVectN

        # magnetometer
        xmag = magVectB[0,0]
        ymag = magVectB[1,0]
        zmag = magVectB[2,0]

        # TODO INSERT NOISE HERE

        return cls(time.time(), xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag)

class Gps(object):

    def __init__(self, time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible):
        self.time = time
        self.fix_type = fix_type
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.eph = eph
        self.epv = epv
        self.vel = vel
        self.cog = cog
        self.satellites_visible = satellites_visible

    def send_to_mav(self, mav):
        try:
            mav.gps_raw_int_send(self.time*sec2msec,
                             self.fix_type,
                             self.lat*rad2degE7, self.lon*rad2degE7, self.alt*m2mm,
                             self.eph*m2cm, self.epv*m2cm, self.vel*m2cm, self.cog*100*rad2deg,
                             self.satellites_visible)
        except struct.error as e:
            print 'mav gps raw int packet data exceeds int bounds'

    @classmethod
    def from_state(cls, state):
        vel = math.sqrt(state.vN*state.vN + state.vE*state.vE)
        cog = math.atan2(state.vE,state.vN) + math.pi

        # TODO INSERT NOISE HERE

        return cls(time = time.time()*sec2msec, fix_type = 3,
                   lat = state.lat, lon = state.lon, alt = state.alt,
                   eph = 1.0, epv = 5.0, vel = vel, cog = cog,
                   satellites_visible = 10)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)
