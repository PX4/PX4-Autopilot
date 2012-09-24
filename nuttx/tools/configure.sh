#!/bin/bash
# configure.sh
#
#   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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
  echo "File \"${configpath}/Make.defs\" does not exist"
  exit 4
fi

if [ ! -r "${configpath}/setenv.sh" ]; then
  echo "File \"${configpath}/setenv.sh\" does not exist"
  exit 5
fi

if [ ! -r "${configpath}/defconfig" ]; then
  echo "File \"${configpath}/defconfig\" does not exist"
  exit 6
fi

# Extract values needed from the defconfig file.  We need:
# (1) The CONFIG_NUTTX_NEWCONFIG setting to know if this is a "new" style
#     configuration, and
# (2) The CONFIG_APPS_DIR to see if there is a configured location for the
#     application directory.

newconfig=`grep CONFIG_NUTTX_NEWCONFIG= "${configpath}/defconfig" | cut -d'=' -f2`

defappdir=y
if [ -z "${appdir}" ]; then
  quoted=`grep "^CONFIG_APPS_DIR=" "${configpath}/defconfig" | cut -d'=' -f2`
  if [ ! -z "${appdir}" ]; then
    appdir=`echo ${quoted} | sed -e "s/\"//g"`
    defappdir=n
  fi
fi

# Check for the apps/ directory in the usual place if appdir was not provided

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

# If appsdir was provided (or discovered) then make sure that the apps/
# directory exists

if [ ! -z "${appdir}" -a ! -d "${TOPDIR}/${appdir}" ]; then
  echo "Directory \"${TOPDIR}/${appdir}\" does not exist"
  exit 7
fi

# Okay... Everything looks good.  Setup the configuration

install -C "${configpath}/Make.defs" "${TOPDIR}/." || \
  { echo "Failed to copy ${configpath}/Make.defs" ; exit 7 ; }
install -C "${configpath}/setenv.sh" "${TOPDIR}/." || \
  { echo "Failed to copy ${configpath}/setenv.sh" ; exit 8 ; }
chmod 755 "${TOPDIR}/setenv.sh"
install -C "${configpath}/defconfig" "${TOPDIR}/.configX" || \
  { echo "Failed to copy ${configpath}/defconfig" ; exit 9 ; }

# If we did not use the CONFIG_APPS_DIR that was in the defconfig config file,
# then append the correct application information to the tail of the .config
# file

if [ "X${defappdir}" = "Xy" ]; then
  sed -i -e "/^CONFIG_APPS_DIR/d" "${TOPDIR}/.configX"
  echo "" >> "${TOPDIR}/.configX"
  echo "# Application configuration" >> "${TOPDIR}/.configX"
  echo "" >> "${TOPDIR}/.configX"
  echo "CONFIG_APPS_DIR=\"$appdir\"" >> "${TOPDIR}/.configX"
fi 

# Copy appconfig file.  The appconfig file will be copied to ${appdir}/.config
# if both (1) ${appdir} is defined and (2) we are not using the new configuration
# (which does not require a .config file in the appsdir.

if [ ! -z "${appdir}" -a "X${newconfig}" != "Xy" ]; then
  if [ ! -r "${configpath}/appconfig" ]; then
    echo "NOTE: No readable appconfig file found in ${configpath}"
  else
    install -C "${configpath}/appconfig" "${TOPDIR}/${appdir}/.config" || \
      { echo "Failed to copy ${configpath}/appconfig" ; exit 10 ; }
  fi
fi

# install the final .configX only if it differs from any existing
# .config file.

install -C "${TOPDIR}/.configX" "${TOPDIR}/.config"
rm -f "${TOPDIR}/.configX"

