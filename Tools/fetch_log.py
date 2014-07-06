#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
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
# 3. Neither the name PX4 nor the names of its contributors may be
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

#
# Log fetcher
#
# Print list of logs:
#     python fetch_log.py
#
# Fetch log:
#     python fetch_log.py sess001/log001.bin
#

import serial, time, sys, os

def wait_for_string(ser, s, timeout=1.0, debug=False):
    t0 = time.time()
    buf = []
    res = []
    n = 0
    while (True):
        c = ser.read()
        if debug:
            sys.stderr.write(c)
        buf.append(c)
        if len(buf) > len(s):
            res.append(buf.pop(0))
            n += 1
            if n % 10000 == 0:
                sys.stderr.write(str(n) + "\n")
        if "".join(buf) == s:
            break
        if timeout > 0.0 and time.time() - t0 > timeout:
            raise Exception("Timeout while waiting for: " + s)
    return "".join(res)

def exec_cmd(ser, cmd, timeout):
    ser.write(cmd + "\n")
    ser.flush()
    wait_for_string(ser, cmd + "\r\n", timeout)
    return wait_for_string(ser, "nsh> \x1b[K", timeout)

def ls_dir(ser, dir, timeout=1.0):
    res = []
    for line in exec_cmd(ser, "ls -l " + dir, timeout).splitlines()[1:]:
        res.append((line[20:], int(line[11:19].strip()), line[1] == "d"))
    return res

def list_logs(ser):
    logs_dir = "/fs/microsd/log"
    res = []
    for d in ls_dir(ser, logs_dir):
        if d[2]:
            sess_dir = d[0][:-1]
            for f in ls_dir(ser, logs_dir + "/" + sess_dir):
                log_file = f[0]
                log_size = f[1]
                res.append(sess_dir + "/" + log_file + "\t" + str(log_size))
    return "\n".join(res)

def fetch_log(ser, fn, timeout):
    cmd = "dumpfile " + fn
    ser.write(cmd + "\n")
    ser.flush()
    wait_for_string(ser, cmd + "\r\n", timeout, True)
    res = wait_for_string(ser, "\n", timeout, True)
    data = []
    if res.startswith("OK"):
        size = int(res.split()[1])
        n = 0
        print "Reading data:"
        while (n < size):
            buf = ser.read(min(size - n, 8192))
            data.append(buf)
            n += len(buf)
            sys.stdout.write(".")
            sys.stdout.flush()
        print
    else:
        raise Exception("Error reading log")
    wait_for_string(ser, "nsh> \x1b[K", timeout)
    return "".join(data)

def main():
    dev = "/dev/tty.usbmodem1"
    ser = serial.Serial(dev, "115200", timeout=0.2)
    if len(sys.argv) < 2:
        print list_logs(ser)
    else:
        log_file = sys.argv[1]
        data = fetch_log(ser, "/fs/microsd/log/" + log_file, 1.0)
        try:
            os.mkdir(log_file.split("/")[0])
        except:
            pass
        fout = open(log_file, "wb")
        fout.write(data)
        fout.close()
    ser.close()

if __name__ == "__main__":
    main()
