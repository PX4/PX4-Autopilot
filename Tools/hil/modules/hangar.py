#!/usr/bin/env python

import time

import aircraft
import sensors

class BasicAircraft(object):

    def __init__(self, attack=None):
        t_now = time.time()
        self.x = aircraft.State.default()
        self.u = aircraft.Controls.default()

        self.imu = sensors.Imu.default()
        self.imu_period = 1.0/200;
        self.t_imu = t_now
        self.imu_count = 0

        self.gps = sensors.Gps.default()
        self.gps_period = 1.0/10;
        self.t_gps = t_now
        self.gps_count = 0

        self.pressure = sensors.Pressure.default()
        self.pressure_period = 1.0/10;
        self.t_pressure = t_now
        self.pressure_count = 0

        self.t_out = t_now

        self.attack = attack

    def update_state(self, fdm):
        self.x = aircraft.State.from_fdm(fdm)
        #self.x.p = 0
        #self.x.q = 0
        #self.x.r = 0
        #self.x.set_attitude(0,0,90*deg2rad)

    def update_controls(self, m):
        self.u = aircraft.Controls.from_mavlink(m)

    def send_controls(self, jsb_console):
        self.u.send_to_jsbsim(jsb_console)

    def send_state(self, mav):
        self.x.send_to_mav(mav)

    def send_imu(self, mav):
        self.imu = sensors.Imu.from_state(self.x, self.attack)
        self.imu.send_to_mav(mav)

    def send_gps(self, mav):
        self.gps = sensors.Gps.from_state(self.x, self.attack)
        self.gps.send_to_mav(mav)

    def send_pressure(self, mav):
        self.pressure = sensors.Pressure.from_state(self.x, self.attack)
        self.pressure.send_to_mav(mav)

    def send_sensors(self, mav):
        t_now = time.time()
        if t_now - self.t_gps > self.gps_period:
            self.t_gps = t_now
            self.send_gps(mav)
            self.gps_count += 1

        t_now = time.time()
        if t_now - self.t_imu > self.imu_period:
            self.t_imu = t_now
            self.send_imu(mav)
            self.imu_count += 1

        t_now = time.time()
        if t_now - self.t_pressure > self.pressure_period:
            self.t_pressure = t_now
            self.send_pressure(mav)
            self.pressure_count += 1

        t_now = time.time()
        if t_now - self.t_out > 1:
            self.t_out = t_now
            print 'imu {0:4d} Hz, gps {1:4d} Hz, pressure {2:4d} Hz\n'.format(
                self.imu_count, self.gps_count, self.pressure_count)
            self.gps_count = 0
            self.imu_count = 0
            self.pressure_count = 0

