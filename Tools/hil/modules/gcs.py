#!/usr/bin/evn python

import time

import pymavlink.mavwp as mavwp
import pymavlink.mavutil as mavutil


class WaypointManager(object):

    def __init__(self, mavfile):
        self.mavfile = mavfile
        self.loader = mavwp.MAVWPLoader(target_system=1,
                                        target_component=190)
        self.count = 0
        self.send_time = 0
        self.recv_time = 0
        self._transition_idle()

    def _transition_idle(self):
        print 'transition idle'
        self.state = 'IDLE'
        self.index = -1

    def _transition_sending_count(self):
        print 'transition sending count'
        self.state = 'SENDING_COUNT'
        self.index = -1
        self.send_time = time.time()

    def _transition_sending_waypoint(self):
        print 'transition sending waypoints'
        self.state = 'SENDING_WAYPOINT'

    def set_waypoints(self, waypoints):
        self.count = self.loader.load(waypoints)

    def send_waypoints(self):
        self._transition_sending_count()

    def send_messages(self):

        if self.state == 'SENDING_COUNT':
            if (time.time() - self.send_time) > 1:
                self.send_time = time.time()
                self.mavfile.waypoint_clear_all_send()
                self.mavfile.waypoint_count_send(self.count)
                print "Sent waypoint count of %u" % self.count

        elif self.state == 'SENDING_WAYPOINT':

            if (time.time() - self.send_time) > 0.3:
                self.send_time = time.time()
                wp = self.loader.wp(self.index)
                self.mavfile.mav.send(wp)
                print "Sent waypoint %u" % (self.index)

                # this seems to prompt vehicle to resend request
                #self.mavfile.waypoint_count_send(self.count)
                pass

    def process_msg(self, msg):

        if msg.get_type() == 'MISSION_REQUEST':
            print 'received request for {0:d} from {1:d}:{2:d}'.format(
                msg.seq, msg.get_srcSystem(),  msg.get_srcComponent())

        # transition to sending waypoints as soon as mission request received
        if self.state == 'SENDING_COUNT':

            if msg.get_type() == 'MISSION_REQUEST':
                self._transition_sending_waypoint()
                self.recv_time = time.time()
                # reprocess this message
                self.process_msg(msg)

        elif self.state == 'SENDING_WAYPOINT':

            if msg.get_type() == 'MISSION_ACK':
                if self.index == self.count - 1:
                    print "Waypoint uploading complete"
                else:
                    print "Error: received waypoint ack before sending all waypoints!"
                self._transition_idle()

            if msg.get_type() == 'MISSION_REQUEST':

                self.recv_time = time.time()

                # should never exceed waypoint count
                if msg.seq >  self.count - 1:
                    print "Error: {0:d} waypoints, but waypoint {1:d} requested".format(
                        self.count,msg.seq)
                    self._transition_idle()
                elif msg.seq == self.index + 1:
                    self.index += 1
