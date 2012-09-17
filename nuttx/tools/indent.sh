#!/bin/sh
############################################################################
# tools/indent.sh
#
#   Copyright (C) 2008, 2010 Gregory Nutt. All rights reserved.
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
# This script uses the Linux 'indent' utility to re-format C source files
# to match the coding style that I use.  It differs from my coding style in that
#
# - I normally put the trailing */ of a multi-line comment on a separate line,
# - I usually align things vertically (like '=' in assignments),
# - indent puts a bogus blank line at the top of the file,
# - I don't like the way it handles nested conditional compilation intermixed with code.
#

# Constants

options="-nbad -bap -bbb -nbbo -nbc -bl -bl2 -bls -nbs -cbi2 -ncdw -nce -ci2 -cli0 -cp40 -ncs -nbfda -nbfde -di1 -nfc1 -fca -i2 -l80 -lp -ppi2 -lps -npcs -pmt -nprs -npsl -saf -sai -sbi2 -saw -sc -sob -nss -nut"

usage="USAGE: $0 <in-file> <out-file>"

# Inputs

infile=$1
outfile=$2

# Verify inputs

if [ -z "$infile" ]; then
    echo "Missing <in-file>"
    echo $usage
    exit 1
fi

if [ ! -r $infile ]; then
    echo "Readable $infile does not exist"
    exit 1
fi

if [ -z "$outfile" ]; then
    echo "Missing <out-file>"
    echo $usage
    exit 1
fi

if [ -f $outfile ]; then
    echo "Removing old $outfile"
    rm $outfile || { echo "Failed to remove $outfile" ; exit 1 ; }
fi

# Perform the indentation

indent $options $infile -o $outfile


