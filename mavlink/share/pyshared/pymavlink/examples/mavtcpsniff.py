#!/usr/bin/env python

'''
connect as a client to two tcpip ports on localhost with mavlink packets.    pass them both directions, and show packets in human-readable format on-screen.

this is useful if 
* you have two SITL instances you want to connect to each other and see the comms.
* you have any tcpip based mavlink happening, and want something better than tcpdump 

hint: 
* you can use netcat/nc to do interesting redorection things with each end if you want to.

Copyright Sept 2012 David "Buzz" Bussenschutt
Released under GNU GPL version 3 or later 
'''

import sys, time, os, struct

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

import mavutil
import mavlinkv10 as mavlink

from optparse import OptionParser
parser = OptionParser("mavfilter.py srcport dstport")

(opts, args) = parser.parse_args()

if len(args) < 1:
    print("Usage: mavfilter.py srcport dstport ")
    sys.exit(1)

srcport =  args[0]
dstport =  args[1]

# gee python string apend is stupid, whatever.  "tcp:localhost:" += srcport  gives: SyntaxError: invalid syntax
msrc = mavutil.mavlink_connection("".join(('tcp:localhost:',srcport)), planner_format=False,
                                  notimestamps=True,
                                  robust_parsing=True)

mdst = mavutil.mavlink_connection("".join(('tcp:localhost:',dstport)), planner_format=False,
                                  notimestamps=True,
                                  robust_parsing=True)


# simple basic byte pass through, no logging or viewing of packets, or analysis etc
#while True:
#  # L -> R
#    m = msrc.recv();
#    mdst.write(m);
#  # R -> L
#    m2 = mdst.recv();
#    msrc.write(m2);


# similar to the above, but with human-readable display of packets on stdout. 
# in this use case we abuse the self.logfile_raw() function to allow 
# us to use the recv_match function ( whch is then calling recv_msg ) , to still get the raw data stream
# which we pass off to the other mavlink connection without any interference.   
# because internally it will call logfile_raw.write() for us.

# here we hook raw output of one to the raw input of the other, and vice versa: 
msrc.logfile_raw = mdst
mdst.logfile_raw = msrc

while True:
  # L -> R
    l = msrc.recv_match();
    if l is not None:
       l_last_timestamp = 0
       if  l.get_type() != 'BAD_DATA':
           l_timestamp = getattr(l, '_timestamp', None)
           if not l_timestamp:
               l_timestamp = l_last_timestamp
           l_last_timestamp = l_timestamp
       
       print("--> %s.%02u: %s\n" % (
           time.strftime("%Y-%m-%d %H:%M:%S",
                         time.localtime(l._timestamp)),
           int(l._timestamp*100.0)%100, l))
           
  # R -> L
    r = mdst.recv_match();
    if r is not None:
       r_last_timestamp = 0
       if r.get_type() != 'BAD_DATA':
           r_timestamp = getattr(r, '_timestamp', None)
           if not r_timestamp:
               r_timestamp = r_last_timestamp
           r_last_timestamp = r_timestamp
   
       print("<-- %s.%02u: %s\n" % (
           time.strftime("%Y-%m-%d %H:%M:%S",
                         time.localtime(r._timestamp)),
           int(r._timestamp*100.0)%100, r))


 