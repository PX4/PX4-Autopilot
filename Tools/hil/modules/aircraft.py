#!/usr/bin/env python

'''
classes for aircraft model
'''

import time, struct, numpy
from math import sin, cos

from constants import *

class Controls(object):
    
    def __init__(self, aileron, elevator, rudder, throttle, aux1, aux2, aux3, aux4,
                mode, nav_mode):
        self.aileron = aileron
        self.elevator = elevator
        self.rudder = rudder
        self.throttle = throttle
        self.aux1 = aux1
        self.aux2 = aux2
        self.aux3 = aux3
        self.aux4 = aux4
        self.mode = mode
        self.nav_mode = nav_mode
        self.C_nb = numpy.identity(3)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)

    def send_to_jsbsim(self, jsb_console):
        jsb_console.send('set %s %s\r\n' % ('fcs/aileron-cmd-norm', self.aileron))
        jsb_console.send('set %s %s\r\n' % ('fcs/elevator-cmd-norm', self.elevator))
        jsb_console.send('set %s %s\r\n' % ('fcs/rudder-cmd-norm', self.rudder))
        jsb_console.send('set %s %s\r\n' % ('fcs/throttle-cmd-norm', self.throttle))

    @classmethod
    def from_mavlink(cls,msg):
        return cls(
            aileron = msg.roll_ailerons,
            elevator = msg.pitch_elevator,
            rudder = msg.yaw_rudder,
            throttle = msg.throttle,
            aux1 = msg.aux1,
            aux2 = msg.aux2,
            aux3 = msg.aux3,
            aux4 = msg.aux4,
            mode = msg.mode,
            nav_mode = msg.nav_mode)

class State(object):
    
    def __init__(self, time,
                 phi, theta, psi,
                 p, q, r,
                 lat, lon, alt,
                 vN, vE, vD,
                 xacc, yacc, zacc):
        self.time = time
        self.set_attitude(phi, theta, psi)
        self.p = p
        self.q = q
        self.r = r
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vN = vN
        self.vE = vE
        self.vD = vD
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc

    def set_attitude(self, phi, theta, psi):
        self.phi = phi
        self.theta = theta 
        self.psi = psi
        cosPhi = cos(phi)
        sinPhi = sin(phi)
        cosThe = cos(theta)
        sinThe = sin(theta)
        cosPsi = cos(psi)
        sinPsi = sin(psi)
        self.C_nb = numpy.matrix([
            [cosThe*cosPsi, 
            -cosPhi*sinPsi + sinPhi*sinThe*cosPsi,
            sinPhi*sinPsi + cosPhi*sinThe*cosPsi],
            [cosThe*sinPsi,
            cosPhi*cosPsi + sinPhi*sinThe*sinPsi,
            -sinPhi*cosPsi + cosPhi*sinThe*sinPsi],
            [-sinThe,
            sinPhi*cosThe,
            cosPhi*cosThe]])

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)

    def send_to_mav(self, mav):
        try:
            mav.hil_state_send(
                self.time*sec2msec, self.phi, self.theta, self.psi,
                self.p, self.q, self.r,
                int(self.lat*rad2degE7), int(self.lon*rad2degE7), int(self.alt*m2mm),
                int(self.vN*m2cm), int(self.vE*m2cm), int(self.vD*m2cm),
                int(self.xacc*mpss2mg), int(self.yacc*mpss2mg), int(self.zacc*mpss2mg))
        except struct.error as e:
            print 'mav hil packet data exceeds int bounds'

    @classmethod
    def from_fdm(cls, fdm):

        # position
        lat = fdm.get('latitude', units='radians')
        lon = fdm.get('longitude', units='radians')
        alt = fdm.get('altitude', units='meters')

        # attitude
        phi = fdm.get('phi', units='radians')
        theta = fdm.get('theta', units='radians')
        psi = fdm.get('psi', units='radians')

        # rotation rates
        phidot = fdm.get('phidot', units='rps')
        thetadot = fdm.get('thetadot', units='rps')
        psidot = fdm.get('psidot', units='rps')

        p = phidot - psidot*sin(theta)
        q = cos(phi)*thetadot + sin(phi)*cos(theta)*psidot
        r = -sin(phi)*thetadot + cos(phi)*cos(theta)*psidot

        # acceleration
        xacc = fdm.get('A_X_pilot', units='mpss')
        yacc = fdm.get('A_Y_pilot', units='mpss')
        zacc = fdm.get('A_Z_pilot', units='mpss')

        # velocitiy
        vN = fdm.get('v_north', units='mps')
        vE = fdm.get('v_east', units='mps')
        vD = fdm.get('v_down', units='mps')

        return cls(time=time.time(),
                             phi=phi, theta=theta, psi=psi,
                             p=p, q=q, r=r,
                             lat=lat, lon=lon, alt=alt,
                             vN=vN, vE=vE, vD=vD,
                             xacc=xacc, yacc=yacc, zacc=zacc)
