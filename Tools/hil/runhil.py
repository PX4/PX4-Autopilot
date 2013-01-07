#!/usr/bin/env python

'''
check bandwidth of link
'''

import sys, struct, time, os
import pymavlink.mavutil as mavutil

class SensorHIL(object):
    ''' This class executes sensor level hil communication '''

    @staticmethod
    def command_line():
        ''' command line parser '''
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--dev', help='device', default='/dev/ttyUSB4')
        parser.add_argument('--baud', help='master port baud rate', default=921600)
        args = parser.parse_args()
        inst = SensorHIL(dev=args.dev, baud=args.baud)
        inst.run()

    def __init__(self, dev, baud):
        ''' default ctor 
        @param dev device
        @param baud baudrate
        '''
        self.master = mavutil.mavlink_connection(dev, baud=baud)

    def enable_hil(self):
        ''' enable hil mode '''
        while not (self.master.base_mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED):
            self.master.set_mode_flag(mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,True)
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
        self.master.set_mode_flag(mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,True)
        print 'mode: {0:x}'.format(self.master.base_mode)

    def run(self):
        ''' main execution loop '''
        t1 = time.time()
        counts = {}
        bytes_sent = 0
        bytes_recv = 0

        self.enable_hil()
        print 'mode: {0:x}'.format(self.master.base_mode)

        # run loop
        while True:
            #self.master.mav.heartbeat_send(1, 1, 1, 1, 1, 1)
            #self.master.mav.sys_status_send(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)
            #self.master.mav.gps_raw_int_send(1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
            #self.master.mav.attitude_send(1, 2, 3, 4, 5, 6, 7)
            #self.master.mav.vfr_hud_send(1, 2, 3, 4, 5, 6)
            roll = 0
            pitch = 0
            yaw = 0
            rollspeed = 0
            pitchspeed = 0
            yawspeed = 0
            lat = 0
            lon = 0
            alt = 0
            vx = 0
            vy = 0
            vz = 0
            xacc = 0
            yacc = 0
            zacc = 0
            self.master.mav.hil_state_send(t1, roll, pitch, yaw,
                                           rollspeed, pitchspeed, yawspeed,
                                           lat, lon, alt,
                                           vx, vy, vz,
                                           xacc, yacc, zacc);
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
                if m == None: break
                if m.get_type() not in counts:
                    counts[m.get_type()] = 0
                counts[m.get_type()] += 1
                if m.get_type() ==  'SERVO_OUTPUT_RAW':
                    print '{} {} {} {} {} {} {} {}'.format(
                        m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw,
                        m.servo5_raw, m.servo6_raw, m.servo7_raw, m.servo8_raw)
            t2 = time.time()
            if t2 - t1 > 1.0:
                print counts
                print("%u sent, %u received, %u errors bwin=%.1f kB/s bwout=%.1f kB/s" % (
                    self.master.mav.total_packets_sent,
                    self.master.mav.total_packets_received,
                    self.master.mav.total_receive_errors,
                    0.001*(self.master.mav.total_bytes_received-bytes_recv)/(t2-t1),
                    0.001*(self.master.mav.total_bytes_sent-bytes_sent)/(t2-t1)))
                bytes_sent = self.master.mav.total_bytes_sent
                bytes_recv = self.master.mav.total_bytes_received
                t1 = t2

if __name__ == "__main__":
    SensorHIL.command_line()
