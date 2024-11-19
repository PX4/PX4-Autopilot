#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
# Serial firmware uploader for the Teensy

import sys
import argparse
import time
import sys
import usb.core
import subprocess

from sys import platform as _platform
from pymavlink import mavutil

try:
    import serial
except ImportError as e:
    print("Failed to import serial: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyserial")
    print("")
    sys.exit(1)

# Define time to use time.time() by default
def _time():
    return time.time()

class uploader(object):

    def __init__(self, portname):
        self.mavlink = mavutil.mavlink_connection(portname)

    def send_reboot(self):
        try:
            self.mavlink.wait_heartbeat()
            self.mavlink.reboot_autopilot(True)
        except:
            pass
        return True

TEENSY_BL_VENDORID = 0x16c0
TEENSY_BL_PRODUCTID = 0x0478

def main():
    # Parse commandline arguments
    parser = argparse.ArgumentParser(description="Firmware uploader for the PX autopilot system.")
    parser.add_argument('--port', action="store", required=True, help="Comma-separated list of serial port(s) to which the FMU may be attached")
    parser.add_argument('--force', action='store_true', default=False, help='Override board type check, or silicon errata checks and continue loading')
    parser.add_argument('--boot-delay', type=int, default=None, help='minimum boot delay to store in flash')
    parser.add_argument('--vendor-id', type=lambda x: int(x,0), default=None, help='PX4 USB vendorid')
    parser.add_argument('--product-id', type=lambda x: int(x,0), default=None, help='PX4 USB productid')
    parser.add_argument('firmware', action="store", nargs='+', help="Firmware file(s)")
    args = parser.parse_args()

    found_bootloader = False

    # find USB devices
    dev = usb.core.find(idVendor=TEENSY_BL_VENDORID, idProduct=TEENSY_BL_PRODUCTID)
    # loop through devices, printing vendor and product ids in decimal and hex
    if dev is not None:
        print("Found teensy bootloader")
        found_bootloader = True
    else:
        dev = usb.core.find(idVendor=args.vendor_id, idProduct=args.product_id)
        if dev is None:
            print("No PX4 Device found try to press the button program push button")
        print("Attempting to reboot into Teensy bootloader...", end="", flush=True)

    try:
        while True:
            portlist = []
            patterns = args.port.split(",")
            # on unix-like platforms use glob to support wildcard ports. This allows
            # the use of /dev/serial/by-id/usb-3D_Robotics on Linux, which prevents the upload from
            # causing modem hangups etc
            if "linux" in _platform or "darwin" in _platform or "cygwin" in _platform:
                import glob
                for pattern in patterns:
                    portlist += glob.glob(pattern)
            else:
                portlist = patterns

            for port in portlist:
                try:
                    if "linux" in _platform:
                        # Linux, don't open Mac OS and Win ports
                        if "COM" not in port and "tty.usb" not in port:
                            up = uploader(port)
                    elif "darwin" in _platform:
                        # OS X, don't open Windows and Linux ports
                        if "COM" not in port and "ACM" not in port:
                            up = uploader(port)
                    elif "cygwin" in _platform:
                        # Cygwin, don't open native Windows COM and Linux ports
                        if "COM" not in port and "ACM" not in port:
                            up = uploader(port)
                    elif "win" in _platform:
                        # Windows, don't open POSIX ports
                        if "/" not in port:
                            up = uploader(port)
                except Exception:
                    # open failed, rate-limit our attempts
                    time.sleep(0.05)

                    # and loop to the next port
                    continue

                while True:
                    up.send_reboot()

                    # wait for the reboot, without we might run into Serial I/O Error 5
                    time.sleep(0.25)

                    # wait for the close, without we might run into Serial I/O Error 6
                    time.sleep(0.3)

                    dev = usb.core.find(idVendor=TEENSY_BL_VENDORID, idProduct=TEENSY_BL_PRODUCTID)
                    # loop through devices, printing vendor and product ids in decimal and hex
                    if dev is not None:
                        print("")
                        print("Found teensy bootloader")
                        found_bootloader = True
                        break

                    print('.', end="", flush=True)

                    if not found_bootloader:
                        # Go to the next port
                        continue

            if(found_bootloader):
                while True:
                    result = subprocess.Popen("teensy_loader_cli -v --mcu=TEENSY41 " + args.firmware[0], shell=True)
                    text = result.communicate()[0]
                    if(result.returncode == 0):
                        sys.exit(0)

            # Delay retries to < 20 Hz to prevent spin-lock from hogging the CPU
            time.sleep(0.05)

    # CTRL+C aborts the upload/spin-lock by interrupt mechanics
    except KeyboardInterrupt:
        print("\n Upload aborted by user.")
        sys.exit(0)

if __name__ == '__main__':
    main()
