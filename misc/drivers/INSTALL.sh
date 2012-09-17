############################################################################
# misc/divers/INSTALL.sh
# Install ALL optional drivers into the NuttX source tree
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

# Directory list

DRIVERS="rtl8187x"

# Parse command arguments

wd=`pwd`
usage="USAGE: $0 [-d|h] <NuttX-path>"

unset nuttxdir
unset debug
unset force

while [ ! -z "$1" ]; do
  case "$1" in
    -d )
      set -x
	  debug="-d"
      ;;
    -f )
	  force="-f"
      ;;
    -h )
      echo "$usage"
      exit 0
      ;;
    *)
      nuttxdir=$1
      ;;
  esac
  shift
done

# Sanity checking

if [ -z "${nuttxdir}" ]; then
  echo "Path to the top-level NuttX directory not provided"
  echo "$usage"
  exit 1
fi

if [ ! -d ${nuttxdir} ]; then
  echo "NuttX directory ${nuttxdir} does not exist"
  exit 2
fi

# Then install each driver

for dir in "$DRIVERS"; do

  # More sanity checking

  if [ ! -d ${dir} ]; then
    echo "No sub-directory ${dir} under ${wd}.  Please CD into the drivers directory first"
    exit 3
  fi
  if [ ! -x ${dir}/INSTALL.sh ]; then
    echo "No executable INSTALL.sh script in ${wd}/${dir}"
    exit 3
  fi

  # Run the driver install script

  ${dir}/INSTALL.sh $debug $force -t "${wd}/${dir}" -n "${nuttxdir}"
done
