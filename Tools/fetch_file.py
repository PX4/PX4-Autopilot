#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2013-2014 PX4 Development Team. All rights reserved.
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

"""Fetch files via nsh console

Usage: python fetch_file.py [-l] [-f] [-d device] [-s speed] [-o out_path] path
\t-l\tList files
\t-f\tOverwrite existing files
\t-d\tSerial device
\t-s\tSerial baudrate
\t-o\tOutput path
\tpath\tPath to list/fetch, if ends with "/" then directory will be fetched recursively"""
from __future__ import print_function

__author__  = "Anton Babushkin"
__version__ = "1.1"

import serial, time, sys, os

def _wait_for_string(ser, s, timeout):
    t0 = time.time()
    buf = []
    res = []
    while (True):
        c = ser.read()
        buf.append(c)
        if len(buf) > len(s):
            res.append(buf.pop(0))
        if "".join(buf) == s:
            break
        if timeout > 0.0 and time.time() - t0 > timeout:
            raise Exception("Timeout while waiting for: " + s)
    return "".join(res)

def _exec_cmd(ser, cmd, timeout):
    ser.write(cmd + "\n")
    ser.flush()
    _wait_for_string(ser, cmd + "\r\n", timeout)
    return _wait_for_string(ser, "nsh> \x1b[K", timeout)

def _ls_dir_raw(ser, dir, timeout):
    return _exec_cmd(ser, "ls -l " + dir, timeout)

def _ls_dir(ser, dir, timeout):
    res = []
    for line in _ls_dir_raw(ser, dir, timeout).splitlines():
        if line == dir + ":":
            continue
        if line.startswith("nsh: ls: no such directory:"):
            raise Exception("No such file: " + dir)
        res.append((line[20:], int(line[11:19].strip()), line[1] == "d"))
    return res

def _get_file(ser, fn, fn_out, force, timeout):
    print("Get %s:" % fn, end=' ')
    if not force:
        # Check if file already exists with the same size
        try:
            os.stat(fn_out)
        except:
            pass
        else:
            print("already fetched, skip")
            return

    cmd = "dumpfile " + fn
    ser.write(cmd + "\n")
    ser.flush()
    _wait_for_string(ser, cmd + "\r\n", timeout)
    res = _wait_for_string(ser, "\n", timeout)
    if res.startswith("OK"):
        # Got correct responce, open temp file
        fn_out_part = fn_out + ".part"
        fout = open(fn_out_part, "wb")

        size = int(res.split()[1])
        sys.stdout.write(" [%i bytes] " % size)
        n = 0
        while (n < size):
            buf = ser.read(min(size - n, 8192))
            n += len(buf)
            sys.stdout.write(".")
            sys.stdout.flush()
            fout.write(buf)
        print(" done")
        fout.close()
        os.rename(fn_out_part, fn_out)
    else:
        raise Exception("Error reading file")
    _wait_for_string(ser, "nsh> \x1b[K", timeout)

def _get_files_in_dir(ser, path, path_out, force, timeout):
    try:
        os.mkdir(path_out)
    except:
        pass
    for fn in _ls_dir(ser, path, timeout):
        path_fn = os.path.join(path, fn[0])
        path_fn_out = os.path.join(path_out, fn[0])
        if fn[2]:
            _get_files_in_dir(ser, path_fn[:-1], path_fn_out[:-1], force, timeout)
        else:
            _get_file(ser, path_fn, path_fn_out, force, timeout)

def _usage():
    print("""Usage: python fetch_file.py [-l] [-f] [-d device] [-s speed] [-o out_path] path
\t-l\tList files
\t-f\tOverwrite existing files
\t-d\tSerial device
\t-s\tSerial baudrate
\t-o\tOutput path
\tpath\tPath to list/fetch, if ends with "/" then directory will be fetched recursively""")

def _main():
    dev = "/dev/tty.usbmodem1"
    speed = "57600"
    cmd = "get"
    path = None
    path_out = None
    force = False
    
    opt = None
    for arg in sys.argv[1:]:
        if opt != None:
            if opt == "d":
                dev = arg
            elif opt == "s":
                speed = arg
            elif opt == "o":
                path_out = arg
            opt = None
        else:
            if arg == "-l":
                cmd = "ls"
            elif arg == "-f":
                force = True
            elif arg == "-d":
                opt = "d"
            elif arg == "-s":
                opt = "s"
            elif arg == "-o":
                opt = "o"
            elif path == None:
                path = arg

    if path == None:
        _usage()
        exit(0)

    # Connect to serial port
    ser = serial.Serial(dev, speed, timeout=0.2)

    timeout = 1.0

    try:
        if cmd == "ls":
            # List directory
            print(_ls_dir_raw(ser, path, timeout))
        elif cmd == "get":
            # Get file(s)
            if path.endswith("/"):
                # Get all files from directory recursively
                if path_out == None:
                    path_out = os.path.split(path[:-1])[1]
                _get_files_in_dir(ser, path[:-1], path_out, force, timeout)
            else:
                # Get one file
                if path_out == None:
                    path_out = os.path.split(path)[1]
                _get_file(ser, path, os.path.split(path)[1], force, timeout)
    except Exception as e:
        print(e)
    
    ser.close()

if __name__ == "__main__":
    _main()
