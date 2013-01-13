#!/usr/bin/env python

import sys, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

import mavlink

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

f = fifo()

# create a mavlink instance, which will do IO on file object 'f'
mav = mavlink.MAVLink(f)

# set the WP_RADIUS parameter on the MAV at the end of the link
mav.param_set_send(7, 1, "WP_RADIUS", 101)

# alternatively, produce a MAVLink_param_set object 
# this can be sent via your own transport if you like
m = mav.param_set_encode(7, 1, "WP_RADIUS", 101)

# get the encoded message as a buffer
b = m.get_msgbuf()

# decode an incoming message
m2 = mav.decode(b)

# show what fields it has
print("Got a message with id %u and fields %s" % (m2.get_msgId(), m2.get_fieldnames()))

# print out the fields
print(m2)
