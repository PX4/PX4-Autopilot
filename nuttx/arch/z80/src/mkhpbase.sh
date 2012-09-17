#/bin/sh
############################################################################
# arch/z80/src/mkhpbase.sh
#
#   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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
#
# This script supports dynamic sizing of HEAP when building with the SDCC
# toolchain.  This script adds up the size of each section of the program.
# This does not account for the alignment of the _CODE area to 0x0100 (hence
# the fudge factor 196 is added).
#
# -- There must be a better way! ---
#
#set -x

MAPFILE=pass1.map

# Verify that pass1.map exists

if [ -e ${MAPFILE} ]; then

	# The map file exists, get a list of the sizes of each section
	# This works for SDCC 2.7.0

	list=`cat ${MAPFILE} | grep "bytes" | sed -e 's/[ \t][ \t]*/ /g' | cut -d' ' -f3`

	# If this is SDCC version 2.6.0, then the list will be empty

	if [ -z "${list}" ]; then

		# This works for SDCC 2.6.0

		list=`cat ${MAPFILE} | grep "SIZE" | sed -e 's/[ \t][ \t]*/ /g' | cut -d' ' -f3`
	fi

	# Did we successfully create the list of section sizes?

	if [ ! -z "${list}" ]; then
		unset result
		first=yes

		# Examine each size in the list

		for size in ${list}; do

			# Skip over zero-length sizes

			if [ "${size}" != "0000" ]; then

				# Each hex size must be prefixed with "0x".  The 
				# First size must also include a left parenthesis

				if [ -z "$first" ]; then
					result="${result} + 0x${size}"
				else
					result="(0x${size}"
					unset first
				fi
			fi
		done

		# Add a fudge factor to guarantee no overlap between the code and
		# the heap and close the expression with a left parenthesis

		echo "${result} + 196)"
	else

		# We could not parse the map file.  Try to generate some meaningful error

		echo "#\"Makefile: Could not parse map file\""
		exit 1
	fi
else
	# pass1.map does not yet exist.  In this case, just output a valid, default heap size

	echo "(CONFIG_HEAP1_END - 8192)"
fi


