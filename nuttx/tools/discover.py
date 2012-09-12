#!/usr/bin/env python
############################################################################
# tools/discover.py
#
#   Copyright (C) 2012 Max Holtzberg. All rights reserved.
#   Author: Max Holtzberg <mh@uvc.de>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

import array
import time
from socket import *

PORT = 96

DISCOVER_PROTO_ID = 0x99
DISCOVER_ALL = 0xff         # 0xff means all devices
DISCOVER_REQUEST = 0x01
DISCOVER_RESPONSE = 0x02
DISCOVER_REQUEST_SIZE = 4
DISCOVER_RESPONSE_SIZE = 35

def check_sum(data):
    chksum = 0
    for c in data[:-1]:
        chksum -= c
    return (chksum & 0xff) == data[-1]

def send_discover(socket):
    cmd = array.array('B', [0] * DISCOVER_REQUEST_SIZE)
    cmd[0] = DISCOVER_PROTO_ID # Tag for identification of the protocol
    cmd[1] = DISCOVER_REQUEST  # Request command
    cmd[2] = DISCOVER_ALL
    chksum = 0
    for c in cmd[:3]:
        chksum -= c;
    cmd[3] = chksum & 0xff
    
    socket.sendto(cmd, ('<broadcast>', PORT))

def read_responses(socket):
    res = []
    response = array.array('B', [0] * DISCOVER_RESPONSE_SIZE)
    try:
        while 1:
            size, src = socket.recvfrom_into(response)
            if (size == DISCOVER_RESPONSE_SIZE
                and response[0] == DISCOVER_PROTO_ID
                and response[1] == DISCOVER_RESPONSE
                and check_sum(response)):
                
                dev = {}
                dev['addr'] = src[0]
                dev['descr'] = response[2:-1].tostring().rstrip('\0')
                res.append(dev)

    except timeout:
        return res

if __name__ == '__main__':
    print 'Sending discover...'

    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('0.0.0.0', PORT))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    s.settimeout(1.0);
    send_discover(s)
    devices = read_responses(s)
    socket.close(s)

    print devices
