############################################################################
#
#   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
# Serial firmware uploader for the PX4FMU bootloader
#
# The PX4 firmware file is a JSON-encoded Python object, containing
# metadata fields and a zlib-compressed base64-encoded firmware image.
#
# The uploader uses the following fields from the firmware file:
#
# image
#	The firmware that will be uploaded.
# image_size
#	The size of the firmware in bytes.
# board_id
#	The board for which the firmware is intended.
# board_revision
#	Currently only used for informational purposes.
#

import sys
import argparse
import binascii
import serial
import os
import struct
import json
import zlib
import base64
import time

from sys import platform as _platform

class firmware(object):
	'''Loads a firmware file'''

	desc = {}
	image = bytearray()

	def __init__(self, path):

		# read the file
		f = open(path, "r")
		self.desc = json.load(f)
		f.close()

		self.image = zlib.decompress(base64.b64decode(self.desc['image']))

		# pad image to 4-byte length
		while ((len(self.image) % 4) != 0):
			self.image += b'\x00'

	def property(self, propname):
		return self.desc[propname]


class uploader(object):
	'''Uploads a firmware file to the PX FMU bootloader'''

	NOP		= chr(0x00)
	OK		= chr(0x10)
	FAILED		= chr(0x11)
	INSYNC		= chr(0x12)
	EOC		= chr(0x20)
	GET_SYNC	= chr(0x21)
	GET_DEVICE	= chr(0x22)
	CHIP_ERASE	= chr(0x23)
	CHIP_VERIFY	= chr(0x24)
	PROG_MULTI	= chr(0x27)
	READ_MULTI	= chr(0x28)
	REBOOT		= chr(0x30)
	
	INFO_BL_REV	= chr(1)	# bootloader protocol revision
	BL_REV		= 2		# supported bootloader protocol 
	INFO_BOARD_ID	= chr(2)	# board type
	INFO_BOARD_REV	= chr(3)	# board revision
	INFO_FLASH_SIZE	= chr(4)	# max firmware size in bytes

	PROG_MULTI_MAX	= 60		# protocol max is 255, must be multiple of 4
	READ_MULTI_MAX	= 60		# protocol max is 255, something overflows with >= 64

	def __init__(self, portname, baudrate):
		# open the port, keep the default timeout short so we can poll quickly
		self.port = serial.Serial(portname, baudrate, timeout=0.25)

	def close(self):
		if self.port is not None:
			self.port.close()

	def __send(self, c):
#		print("send " + binascii.hexlify(c))
		self.port.write(str(c))

	def __recv(self, count = 1):
		c = self.port.read(count)
		if (len(c) < 1):
			raise RuntimeError("timeout waiting for data")
#		print("recv " + binascii.hexlify(c))
		return c

	def __getSync(self):
		self.port.flush()
		c = self.__recv()
		if (c != self.INSYNC):
			raise RuntimeError("unexpected 0x%x instead of INSYNC" % ord(c))
		c = self.__recv()
		if (c != self.OK):
			raise RuntimeError("unexpected 0x%x instead of OK" % ord(c))

	# attempt to get back into sync with the bootloader
	def __sync(self):
		# send a stream of ignored bytes longer than the longest possible conversation
		# that we might still have in progress
#		self.__send(uploader.NOP * (uploader.PROG_MULTI_MAX + 2))
		self.port.flushInput()
		self.__send(uploader.GET_SYNC 
				+ uploader.EOC)
		self.__getSync()
		
	def __trySync(self):
		c = self.__recv()
		if (c != self.INSYNC):
			#print("unexpected 0x%x instead of INSYNC" % ord(c))
			return False;
		c = self.__recv()
		if (c != self.OK):
			#print("unexpected 0x%x instead of OK" % ord(c))
			return False
		return True

	# send the GET_DEVICE command and wait for an info parameter
	def __getInfo(self, param):
		self.__send(uploader.GET_DEVICE + param + uploader.EOC)
		raw = self.__recv(4)
		self.__getSync()
		value = struct.unpack_from('<I', raw)
		return value[0]

	# send the CHIP_ERASE command and wait for the bootloader to become ready
	def __erase(self):
		self.__send(uploader.CHIP_ERASE 
				+ uploader.EOC)
		# erase is very slow, give it 10s
		deadline = time.time() + 10
		while time.time() < deadline:
			try:
				self.__getSync()
				return
			except RuntimeError as ex:
				# we timed out, that's OK
				continue

		raise RuntimeError("timed out waiting for erase")

	# send a PROG_MULTI command to write a collection of bytes
	def __program_multi(self, data):
		self.__send(uploader.PROG_MULTI
				+ chr(len(data)))
		self.__send(data)
		self.__send(uploader.EOC)
		self.__getSync()
		
	# verify multiple bytes in flash
	def __verify_multi(self, data):
		self.__send(uploader.READ_MULTI
				+ chr(len(data))
				+ uploader.EOC)
		programmed = self.__recv(len(data))
		if (programmed != data):
			print("got    " + binascii.hexlify(programmed))
			print("expect " + binascii.hexlify(data))
			return False
		self.__getSync()
		return True
		
	# send the reboot command
	def __reboot(self):
		self.__send(uploader.REBOOT)
		self.port.flush()

	# split a sequence into a list of size-constrained pieces
	def __split_len(self, seq, length):
    		return [seq[i:i+length] for i in range(0, len(seq), length)]

	# upload code
	def __program(self, fw):
		code = fw.image
		groups = self.__split_len(code, uploader.PROG_MULTI_MAX)
		for bytes in groups:
			self.__program_multi(bytes)

	# verify code
	def __verify(self, fw):
		self.__send(uploader.CHIP_VERIFY
				+ uploader.EOC)
		self.__getSync()
		code = fw.image
		groups = self.__split_len(code, uploader.READ_MULTI_MAX)
		for bytes in groups:
			if (not self.__verify_multi(bytes)):
				raise RuntimeError("Verification failed")

	# get basic data about the board
	def identify(self):
		# make sure we are in sync before starting
		self.__sync()

		# get the bootloader protocol ID first
		bl_rev = self.__getInfo(uploader.INFO_BL_REV)
		if bl_rev != uploader.BL_REV:
			raise RuntimeError("Bootloader protocol mismatch")

		self.board_type = self.__getInfo(uploader.INFO_BOARD_ID)
		self.board_rev = self.__getInfo(uploader.INFO_BOARD_REV)
		self.fw_maxsize = self.__getInfo(uploader.INFO_FLASH_SIZE)

	# upload the firmware
	def upload(self, fw):
		# Make sure we are doing the right thing
		if self.board_type != fw.property('board_id'):
			raise RuntimeError("Firmware not suitable for this board")
		if self.fw_maxsize < fw.property('image_size'):
			raise RuntimeError("Firmware image is too large for this board")

		print("erase...")
		self.__erase()

		print("program...")
		self.__program(fw)

		print("verify...")
		self.__verify(fw)

		print("done, rebooting.")
		self.__reboot()
		self.port.close()
	

# Parse commandline arguments
parser = argparse.ArgumentParser(description="Firmware uploader for the PX autopilot system.")
parser.add_argument('--port', action="store", required=True, help="Serial port(s) to which the FMU may be attached")
parser.add_argument('--baud', action="store", type=int, default=115200, help="Baud rate of the serial port (default is 115200), only required for true serial ports.")
parser.add_argument('firmware', action="store", help="Firmware file to be uploaded")
args = parser.parse_args()

# Load the firmware file
fw = firmware(args.firmware)
print("Loaded firmware for %x,%x, waiting for the bootloader..." % (fw.property('board_id'), fw.property('board_revision')))

# Spin waiting for a device to show up
while True:
	for port in args.port.split(","):

		#print("Trying %s" % port)

		# create an uploader attached to the port
		try:
			if "linux" in _platform:
			# Linux, don't open Mac OS and Win ports
				if not "COM" in port and not "tty.usb" in port:
					up = uploader(port, args.baud)
			elif "darwin" in _platform:
				# OS X, don't open Windows and Linux ports
				if not "COM" in port and not "ACM" in port:
					up = uploader(port, args.baud)
			elif "win" in _platform:
				# Windows, don't open POSIX ports
				if not "/" in port:
					up = uploader(port, args.baud)
		except:
			# open failed, rate-limit our attempts
			time.sleep(0.05)

			# and loop to the next port
			continue

		# port is open, try talking to it
		try:
			# identify the bootloader
			up.identify()
			print("Found board %x,%x on %s" % (up.board_type, up.board_rev, port))

		except:
			# most probably a timeout talking to the port, no bootloader
			continue

		try:
			# ok, we have a bootloader, try flashing it
			up.upload(fw)

		except RuntimeError as ex:

			# print the error
			print("ERROR: %s" % ex.args)

		finally:
			# always close the port
			up.close()

		# we could loop here if we wanted to wait for more boards...
		sys.exit(0)
