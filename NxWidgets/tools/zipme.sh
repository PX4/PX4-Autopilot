#!/bin/bash
#################################################################################
# NxWidgets/tools/zipme.sh
#
#   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
# 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
#    me be used to endorse or promote products derived from this software
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
#################################################################################

#set -x

WD=`pwd`
VERSION=$1

TAR="tar cvf"
ZIP=gzip

# Make sure we know what is going on

if [ -z ${VERSION} ] ; then
   echo "You must supply a version like xx.yy as a parameter"
   exit 1;
fi

# Find the directory we were executed from and were we expect to
# see the directory to tar up

MYNAME=`basename $0`

if [ -x ${WD}/${MYNAME} ] ; then
   NXWIDGETS=`dirname ${WD}`
else
   if [ -x ${WD}/tools/${MYNAME} ] ; then
     NXWIDGETS=${WD}
   else
     echo "You must cd into the NxWidgets or NxWidgets/tools directory to execute this script."
     exit 1
   fi
fi

# Get the NxWidgets directory name and the path to the parent directory

NXWIDGETSDIR=`basename ${NXWIDGETS}`
PROJECTS=`dirname ${NXWIDGETS}`

# The name of the directory must match the version number

if [ "X${NXWIDGETSDIR}" != "XNxWidgets-${VERSION}" ]; then
   echo "Expected directory name to be NxWidgets-${VERSION} found ${NXWIDGETSDIR}"
   exit 1
fi

cd ${PROJECTS} || \
   { echo "Failed to cd to ${PROJECTS}" ; exit 1 ; }

if [ ! -d ${NXWIDGETSDIR} ] ; then
   echo "${PROJECTS}/${NXWIDGETSDIR} does not exist!"
   exit 1
fi

TAR_NAME=NxWidgets-${VERSION}.tar
ZIP_NAME=${TAR_NAME}.gz

# Prepare the NxWidgets directory -- Remove editor garbage

find ${NXWIDGETSDIR} -name '*~' -exec rm -f '{}' ';' || \
      { echo "Removal of emacs garbage failed!" ; exit 1 ; }

find ${NXWIDGETSDIR} -name '#*' -exec rm -f '{}' ';' || \
      { echo "Removal of VI garbage failed!" ; exit 1 ; }

find ${NXWIDGETSDIR} -name '*.swp' -exec rm -f '{}' ';' || \
      { echo "Removal of VI garbage failed!" ; exit 1 ; }

# Perform a full clean for the distribution

make -C ${NXWIDGETSDIR} distclean

# Remove any previous tarballs

if [ -f ${TAR_NAME} ] ; then
    echo "Removing ${PROJECTS}/${TAR_NAME}"
    rm -f ${TAR_NAME} || \
        { echo "rm ${TAR_NAME} failed!" ; exit 1 ; }
fi

if [ -f ${ZIP_NAME} ] ; then
    echo "Removing ${PROJECTS}/${ZIP_NAME}"
    rm -f ${ZIP_NAME} || \
        { echo "rm ${ZIP_NAME} failed!" ; exit 1 ; }
fi

# Then zip it

${TAR} ${TAR_NAME} ${NXWIDGETSDIR} || \
    { echo "tar of ${NXWIDGETSDIR} failed!" ; exit 1 ; }
${ZIP} ${TAR_NAME} || \
    { echo "zip of ${TAR_NAME} failed!" ; exit 1 ; }
