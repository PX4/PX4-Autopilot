#!/bin/bash
# configure.sh
#
#   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

WD=`pwd`
TOPDIR="${WD}/.."
USAGE="

USAGE: ${0} [-d] [-a <app-dir>] <board-name>/<config-name>

Where:
  <board-name> is the name of the board in the configs directory
  <config-name> is the name of the board configuration sub-directory
  <add-dir> is the path to the apps/ directory, relative to the nuttx directory

"

# Parse command arguments

unset boardconfig
unset appdir

while [ ! -z "$1" ]; do
  case "$1" in
    -d )
      set -x
      ;;
    -h )
      echo "$USAGE"
      exit 0
      ;;
    -a )
      shift
      appdir=$1
      ;;
    *)
      if [ ! -z "${boardconfig}" ]; then
        echo ""
        echo "<board/config> defined twice"
        echo "$USAGE"
        exit 1
      fi
      boardconfig=$1
      ;;
  esac
  shift
done

# Sanity checking

if [ -z "${boardconfig}" ]; then
  echo ""
  echo "Missing <board/config> argument"
  echo "$USAGE"
  exit 2
fi

configpath=${TOPDIR}/configs/${boardconfig}
if [ ! -d "${configpath}" ]; then
  echo "Directory ${configpath} does not exist.  Options are:"
  echo ""
  echo "Select one of the following options for <board-name>:"
  configlist=`find ${TOPDIR}/configs -name defconfig`
  for defconfig in $configlist; do
    config=`dirname $defconfig | sed -e "s,${TOPDIR}/configs/,,g"`
    echo "  $config"
  done
  echo ""
  echo "$USAGE"
  exit 3
fi

if [ ! -r "${configpath}/Make.defs" ]; then
  echo "File ${configpath}/Make.defs does not exist"
  exit 4
fi

if [ ! -r "${configpath}/setenv.sh" ]; then
  echo "File ${configpath}/setenv.sh does not exist"
  exit 5
fi

if [ ! -r "${configpath}/defconfig" ]; then
  echo "File ${configpath}/defconfig does not exist"
  exit 6
fi

# Check for the apps/ dir in the usual place if appdir was not provided

if [ -z "${appdir}" ]; then

  # Check for a version file

  unset CONFIG_VERSION_STRING
  if [ -x "${TOPDIR}/.version" ]; then
    . "${TOPDIR}/.version"
  fi

  # Check for an unversioned apps/ directory
 
  if [ -d "${TOPDIR}/../apps" ]; then
    appdir="../apps"

  else
    # Check for a versioned apps/ directory

    if [ -d "${TOPDIR}/../apps-${CONFIG_VERSION_STRING}" ]; then
      appdir="../apps-${CONFIG_VERSION_STRING}"
    fi 
  fi
fi

# Okay... setup the configuration

cp -f "${configpath}/Make.defs" "${TOPDIR}/." || \
  { echo "Failed to copy ${configpath}/Make.defs" ; exit 7 ; }
cp -f "${configpath}/setenv.sh" "${TOPDIR}/." || \
  { echo "Failed to copy ${configpath}/setenv.sh" ; exit 8 ; }
chmod 755 "${TOPDIR}/setenv.sh"
cp -f "${configpath}/defconfig" "${TOPDIR}/.config" || \
  { echo "Failed to copy ${configpath}/defconfig" ; exit 9 ; }

# Copy option appconfig

if [ ! -z "${appdir}" ]; then
  if [ ! -r "${configpath}/appconfig" ]; then
    echo "NOTE: No readable appconfig file found in ${configpath}"
  else
    cp -f "${configpath}/appconfig" "${TOPDIR}/${appdir}/.config" || \
      { echo "Failed to copy ${configpath}/appconfig" ; exit 10 ; }

    echo "" >> "${TOPDIR}/.config"
    echo "# Application configuration" >> "${TOPDIR}/.config"
    echo "" >> "${TOPDIR}/.config"
    echo "CONFIG_APPS_DIR=\"$appdir\"" >> "${TOPDIR}/.config"
  fi
fi

