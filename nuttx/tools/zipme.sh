#!/bin/bash
# zipme.sh
#
#   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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
# see the directories to tar up

MYNAME=`basename $0`

if [ -x ${WD}/${MYNAME} ] ; then
   TRUNKDIR="${WD}/../.."
else
   if [ -x ${WD}/tools/${MYNAME} ] ; then
     TRUNKDIR="${WD}/.."
   else
     if [ -x ${WD}/nuttx-${VERSION}/tools/${MYNAME} ] ; then
       TRUNKDIR="${WD}"
     else
       echo "You must cd into the NUTTX directory to execute this script."
       exit 1
     fi
   fi
fi

# Get the NuttX directory names and the path to the parent directory

NUTTX=${TRUNKDIR}/nuttx-${VERSION}
APPDIR=${TRUNKDIR}/apps-${VERSION}

# Make sure that the versioned directory exists

if [ ! -d ${TRUNKDIR} ]; then
   echo "Directory ${TRUNKDIR} does not exist"
   exit 1
fi

cd ${TRUNKDIR} || \
   { echo "Failed to cd to ${TRUNKDIR}" ; exit 1 ; }

if [ ! -d nuttx-${VERSION} ] ; then
   echo "Directory ${TRUNKDIR}/nuttx-${VERSION} does not exist!"
   exit 1
fi

if [ ! -d apps-${VERSION} ] ; then
   echo "Directory ${TRUNKDIR}/apps-${VERSION} does not exist!"
   exit 1
fi

# Create the versioned tarball names

NUTTX_TARNAME=nuttx-${VERSION}.tar
APPS_TARNAME=apps-${VERSION}.tar
NUTTX_ZIPNAME=${NUTTX_TARNAME}.gz
APPS_ZIPNAME=${APPS_TARNAME}.gz

# Prepare the nuttx directory -- Remove editor garbage

find ${TRUNKDIR} -name '*~' -exec rm -f '{}' ';' || \
      { echo "Removal of emacs garbage failed!" ; exit 1 ; }
find ${TRUNKDIR} -name '*.swp' -exec rm -f '{}' ';' || \
      { echo "Removal of VI garbage failed!" ; exit 1 ; }

# Make sure that versioned copies of the certain files are in place

cd ${NUTTX}/Documentation || \
   { echo "Failed to cd to ${NUTTX}/Documentation" ; exit 1 ; }

cp -f ../TODO TODO.txt
cp -f ../ChangeLog ChangeLog.txt

# Write a version file into the NuttX directoy.  The syntax of file is such that it
# may be sourced by a bash script or included by a Makefile.

VERSIONSH=${NUTTX}/tools/version.sh
if [ ! -x "${VERSIONSH}" ]; then
	echo "No executable script was found at: ${VERSIONSH}"
	exit 1
fi
${VERSIONSH} -v ${VERSION} ${NUTTX}/.version || \
	{ echo "${VERSIONSH} failed"; cat ${NUTTX}/.version; exit 1; }
chmod 755 ${NUTTX}/.version

# Perform a full clean for the distribution

cd ${TRUNKDIR} || \
   { echo "Failed to cd to ${TRUNKDIR}" ; exit 1 ; }

make -C ${NUTTX} distclean

# Remove any previous tarballs

if [ -f ${NUTTX_TARNAME} ] ; then
   echo "Removing ${TRUNKDIR}/${NUTTX_TARNAME}"
   rm -f ${NUTTX_TARNAME} || \
      { echo "rm ${NUTTX_TARNAME} failed!" ; exit 1 ; }
fi

if [ -f ${NUTTX_ZIPNAME} ] ; then
   echo "Removing ${TRUNKDIR}/${NUTTX_ZIPNAME}"
   rm -f ${NUTTX_ZIPNAME} || \
      { echo "rm ${NUTTX_ZIPNAME} failed!" ; exit 1 ; }
fi

if [ -f ${APPS_TARNAME} ] ; then
   echo "Removing ${TRUNKDIR}/${APPS_TARNAME}"
   rm -f ${APPS_TARNAME} || \
      { echo "rm ${APPS_TARNAME} failed!" ; exit 1 ; }
fi

if [ -f ${APPS_ZIPNAME} ] ; then
   echo "Removing ${TRUNKDIR}/${APPS_ZIPNAME}"
   rm -f ${APPS_ZIPNAME} || \
      { echo "rm ${APPS_ZIPNAME} failed!" ; exit 1 ; }
fi

# Then tar and zip-up the directories

cd ${TRUNKDIR} || \
   { echo "Failed to cd to ${TRUNKDIR}" ; exit 1 ; }

${TAR} ${NUTTX_TARNAME} nuttx-${VERSION} || \
      { echo "tar of ${NUTTX_TARNAME} failed!" ; exit 1 ; }
${ZIP} ${NUTTX_TARNAME} || \
      { echo "zip of ${NUTTX_TARNAME} failed!" ; exit 1 ; }

${TAR} ${APPS_TARNAME} apps-${VERSION} || \
      { echo "tar of ${APPS_TARNAME} failed!" ; exit 1 ; }
${ZIP} ${APPS_TARNAME} || \
      { echo "zip of ${APPS_TARNAME} failed!" ; exit 1 ; }

cd ${NUTTX}

