#! /usr/bin/python

import serial, time
import subprocess
from subprocess import call, Popen
import re


# './Tools/px_uploader.py --port $USB_DEVICE --baud-flightstack 115200 --baud-bootloader 115200 ./build/px4fmu-v4_default/nuttx_px4fmu-v4_default.px4'

def flash_firmware():

    flash_cmd = './Tools/px_uploader.py --port /dev/ttyACM0 \
                                        --baud-flightstack 115200 \
                                        --baud-bootloader 115200 \
                                        ./build/px4fmu-v4_default/px4fmu-v4_default.px4'

    proc = subprocess.Popen(flash_cmd, shell= True, stdout=subprocess.PIPE, cwd="../../")

    erased = False
    programmed = False
    verified = False

    for line in proc.stdout:
        print(line)
        if "Erase  : [====================] 100.0%" in line:
            erased = True
        if "Program: [====================] 100.0%" in line:
            programmed = True
        if "Verify : [====================] 100.0%" in line:
            verified = True

    assert erased == True, "Error during erasing"
    assert programmed == True, "Error during programming"
    assert verified == True, "Error during verifying"

def main():
    flash_firmware()


if __name__ == "__main__":
   main()

