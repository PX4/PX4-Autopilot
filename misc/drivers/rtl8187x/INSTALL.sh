############################################################################
# nuttx/rtl8187x/INSTALL.sh
# Install the GPLv2 RTL8187x driver into the NuttX source tree
#
#   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

# List of files to install and where to install them

files="\
rtl8187x.c
rtl8187x.h
"
dest="drivers/usbhost"

# Parse command arguments

usage="USAGE: $0 [-d|f|h] -t <driver-dir> -n <nuttx-dir>"

unset topdir
unset nuttxdir
unset force

while [ ! -z "$1" ]; do
  case "$1" in
    -d )
      set -x
      ;;
    -f )
      force=y
      ;;
    -t )
	  shift
	  topdir=$1
      ;;
    -n )
	  shift
	  nuttxdir=$1
      ;;
    -h )
      echo "$usage"
      exit 0
      ;;
    *)
	  echo "Unrecognized option: $1"
	  echo "$usage"
	  exit 1
      ;;
  esac
  shift
done

# Sanity checking 

if [ -z "${nuttxdir}" ]; then
  echo "Path to the top-level NuttX directory not provided"
  echo "$usage"
  exit 2
fi

if [ -z "${topdir}" ]; then
  echo "Path to the top-level misc/drivers directory not provided"
  echo "$usage"
  exit 3
fi

if [ ! -d ${nuttxdir} ]; then
  echo "NuttX directory ${nuttxdir} does not exist"
  exit 4
fi

if [ ! -d ${topdir} ]; then
  echo "misc/drivers directory ${topdir} does not exist"
  exit 5
fi

if [ ! -d ${nuttxdir}/${dest} ]; then
  echo "NuttX driver directory ${nuttxdir}/${dest} does not exist"
  exit 6
fi

echo "Installing the RTL8187x driver to ${nuttxdir}/${dest}"

# Copy the files

for file in ${files}; do
  if [ ! -r ${topdir}/${file} ]; then
    echo "No readable source driver file ${topdir}/${file}"
	exit 7
  fi
  if [ -f ${nuttxdir}/${dest}/${file} ]; then
    echo "Driver file ${nuttxdir}/${dest}/${file} already exists"
    if [ "X${force}" = "Xy" ]; then
      echo "Removing old file ${nuttxdir}/${dest}/${file}"
      rm -f ${nuttxdir}/${dest}/${file} || \
        { echo "ERROR: failed to remove ${nuttxdir}/${dest}/${file}"; exit 8; }
    else
      echo "Please remove that file and re-start the installation"
      echo "Or use the -f option to force over writing of the file"
	  exit 8
    fi
  fi
  cp ${topdir}/${file} ${nuttxdir}/${dest}/${file} || \
    { echo "ERROR: failed to copy ${topdir}/${file} to ${nuttxdir}/${dest}/${file}"; exit 9; }
done
