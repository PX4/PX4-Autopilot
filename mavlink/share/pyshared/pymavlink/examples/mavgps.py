#!/usr/bin/python
"""
Allows connection of the uBlox u-Center software to
a uBlox GPS device connected to a PX4 or Pixhawk device,
using Mavlink's SERIAL_CONTROL support to route serial
traffic to/from the GPS, and exposing the data to u-Center
via a local TCP connection.

@author: Matthew Lloyd (github@matthewlloyd.net)
"""

from pymavlink import mavutil
from optparse import OptionParser
import socket


def main():
    parser = OptionParser("mavgps.py [options]")
    parser.add_option("--mavport", dest="mavport", default=None,
                      help="Mavlink port name")
    parser.add_option("--mavbaud", dest="mavbaud", type='int',
                      help="Mavlink port baud rate", default=115200)
    parser.add_option("--devnum", dest="devnum", default=2, type='int',
                      help="PX4 UART device number (defaults to GPS port)")
    parser.add_option("--devbaud", dest="devbaud", default=38400, type='int',
                      help="PX4 UART baud rate (defaults to u-Blox GPS baud)")
    parser.add_option("--tcpport", dest="tcpport", default=1001, type='int',
                      help="local TCP port (defaults to 1001)")
    parser.add_option("--debug", dest="debug", default=0, type='int',
                      help="debug level")
    parser.add_option("--buffsize", dest="buffsize", default=128, type='int',
                      help="buffer size")
    (opts, args) = parser.parse_args()

    if opts.mavport is None:
        parser.error("You must specify a Mavlink serial port (--mavport)")

    print "Connecting to MAVLINK..."
    mav_serialport = mavutil.MavlinkSerialPort(
        opts.mavport, opts.mavbaud,
        devnum=opts.devnum, devbaud=opts.devbaud, debug=opts.debug)

    listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listen_sock.bind(('127.0.0.1', opts.tcpport))
    listen_sock.listen(1)

    print "Waiting for a TCP connection."
    print "Use tcp://localhost:%d in u-Center." % opts.tcpport
    conn_sock, addr = listen_sock.accept()
    conn_sock.setblocking(0)  # non-blocking mode
    print "TCP connection accepted. Use Ctrl+C to exit."

    while True:
        try:
            data = conn_sock.recv(opts.buffsize)
            if data:
                if opts.debug >= 1:
                    print '>', len(data)
                mav_serialport.write(data)
        except socket.error:
            pass

        data = mav_serialport.read(opts.buffsize)
        if data:
            if opts.debug >= 1:
                print '<', len(data)
            conn_sock.send(data)


if __name__ == '__main__':
    main()
