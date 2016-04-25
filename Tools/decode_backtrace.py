#!/usr/bin/env python
############################################################################
#
# Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

import os
import subprocess

# The PX4_BACKTRACE() macro can be used to generate backtraces in the
# running code. These provide the addresses only. This utility can be
# used to decode the backtrace into a readable form:
#
# Example:
# If the following lines were pasted into the shell after running decode_backtrace.py
#
# INFO  Backtrace: 10
# INFO  ./mainapp(px4_backtrace+0x27) [0x42b212]
# INFO  ./mainapp() [0x42d608]
# INFO  ./mainapp() [0x42d57e]
# INFO  ./mainapp() [0x4ba48d]
#
# The output would be:
#
# 0x42b212 px4_backtrace (0x42b1eb)
# 0x42d608 ACCELSIM::devRead(void*, unsigned long) (0x42d600)
# 0x42d57e ACCELSIM::start() (0x42d578)
# 0x4ba48d DriverFramework::DevObj::addHandle(DriverFramework::DevHandle&) (0x4ba42e)
#
# The output format is:
# <backtrace address> <function name> (<function address>)


def usage():
	msg = """
Usage: Tools/decode_backtrace.py <builddir>

This will load the symbols for <builddir>/src/firmware/posix/mainapp
The user just needs to copy and paste the backtrace into the terminal
where decode_backtrace.py is running.

"""
	print msg

funcaddr = []
func = []

# Load the symbols from the binary
def load_symbol_map():
	output = subprocess.check_output(["nm", "-p", "-C", os.sys.argv[1]+"/src/firmware/posix/mainapp"])
	data = output.split("\n")
	data.sort()

	i = 0
	for line in data:
		if line.startswith("0"):
			funcaddr.append(int(line[0:16],16))
			func.append(line[19:])
			#print(hex(funcaddr[i]), func[i])
			i = i+1

# Find the function for the specified call stack address
def lookup(addr):
	i=1
	while i < len(funcaddr) and addr > funcaddr[i]:
		i=i+1
	if i >= len(funcaddr):
		return -1

	# return the function before this one
	return i-1

if len(os.sys.argv) != 2:
	usage()
	raise SystemExit

load_symbol_map()

print
print "Paste the backtrace here. Use Ctrl-C to exit."
print
while 1:
	# Wait for user to paste the backtrace into the shell
	line = raw_input()
	tmp = line.split("[")
	if len(tmp) == 1:
		if len(line.split("Backtrace:")) > 1:
			print
		continue
	addrstr = tmp[1].split("]")[0]
	addr = int(addrstr, 16)
	idx = lookup(addr)
	if idx >=  0:
		print hex(addr), func[idx], "({0})".format(hex(funcaddr[idx]))

