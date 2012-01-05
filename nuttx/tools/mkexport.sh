#!/bin/bash
# tools/mkexport.sh
#
#   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

# TODO:
# 1. This script assumes the host archiver ar may not be appropriate for
#    non-GCC toolchains
# 2. For the kernel build, the user libriars should be built into some
#    libuser.a.  The list of user libraries would have to accepted with
#    some new argument, perhaps -u.

# Get the input parameter list

USAGE="USAGE: $0 [-d] [-z] [-w|wy|wn] -t <top-dir> [-x <lib-ext>] -l \"lib1 [lib2 [lib3 ...]]\""
unset TOPDIR
unset LIBLIST
unset TGZ
WINTOOL=n
LIBEXT=.a

while [ ! -z "$1" ]; do
	case $1 in
 		-d )
			set -x
			;;
 		-l )
			shift
			LIBLIST=$1
			;;
		-wy )
			WINTOOL=y
			;;
		-w | -wn )
			WINTOOL=n
			;;
		-t )
			shift
			TOPDIR=$1
			;;
 		-x )
			shift
			LIBEXT=$1
			;;
 		-z )
			TGZ=y
			;;
		-h )
			echo $USAGE
			exit 0
			;;
		* )
			echo "Unrecognized argument: $1"
			echo $USAGE
			exit 1
			;;
		esac
	shift
done

# Check arguments

if [ -z "${TOPDIR}" -o -z "${LIBLIST}" ]; then
	echo "MK: Missing required arguments"
	echo $USAGE
	exit 1
fi

if [ ! -d "${TOPDIR}" ]; then
	echo "MK: Directory ${TOPDIR} does not exist"
	exit 1
fi

# Get the version string

if [ ! -f "${TOPDIR}/.version" ]; then
	echo "MK: File ${TOPDIR}/.version does not exist"
	exit 1
fi

source "${TOPDIR}/.version"
if [ ! -z "${CONFIG_VERSION_STRING}" -a "${CONFIG_VERSION_STRING}" != "0.0" ]; then
	VERSION="-${CONFIG_VERSION_STRING}"
fi

# Create the export directory

EXPORTSUBDIR="nuttx-export${VERSION}"
EXPORTDIR="${TOPDIR}/${EXPORTSUBDIR}"

# If the export directory already exists, then remove it and create a new one

if [ -d "${EXPORTDIR}" ]; then
	echo "MK: Removing old export directory"
	rm -rf "${EXPORTDIR}"
fi

# Remove any possible previous results

rm -f "${EXPORTDIR}.tar"
rm -f "${EXPORTDIR}.zip"
rm -f "${EXPORTDIR}.tar.gz"

# Create the export directory and some of its subdirectories

mkdir "${EXPORTDIR}" || { echo "MK: 'mkdir ${EXPORTDIR}' failed"; exit 1; }
mkdir "${EXPORTDIR}/startup" || { echo "MK: 'mkdir ${EXPORTDIR}/startup' failed"; exit 1; }
mkdir "${EXPORTDIR}/libs" || { echo "MK: 'mkdir ${EXPORTDIR}/libs' failed"; exit 1; }
mkdir "${EXPORTDIR}/build" || { echo "MK: 'mkdir ${EXPORTDIR}/build' failed"; exit 1; }
mkdir "${EXPORTDIR}/arch" || { echo "MK: 'mkdir ${EXPORTDIR}/arch' failed"; exit 1; }

# Verify that we have a Make.defs file.

if [ ! -f "${TOPDIR}/Make.defs" ]; then
	echo "MK: Directory ${TOPDIR}/Make.defs does not exist"
	exit 1
fi

# Copy the Make.defs files, but disable windows path conversions

grep -v "WINTOOL[ \t]*=[ \t]y" "${TOPDIR}/Make.defs"  > "${EXPORTDIR}/Make.defs"

# Extract information from the Make.defs file.  A Makefile can do this best

make -C "${TOPDIR}/tools" -f Makefile.export TOPDIR="${TOPDIR}" EXPORTDIR="${EXPORTDIR}"
source "${EXPORTDIR}/makeinfo.sh"
rm -f "${EXPORTDIR}/makeinfo.sh"
rm -f "${EXPORTDIR}/Make.defs"

# Verify the build info that we got from makeinfo.sh

if [ ! -d "${ARCHDIR}" ]; then
	echo "MK: Directory ${ARCHDIR} does not exist"
	exit 1
fi

# Is there a linker script in this configuration?

if [ ! -z "${LDPATH}" ]; then

	# Apparently so.  Verify that the script exists

	if [ ! -f "${LDPATH}" ]; then
		echo "MK: File ${LDPATH} does not exist"
		exit 1
	fi

	# Copy the linker script

	cp -p "${LDPATH}" "${EXPORTDIR}/build/." || \
		{ echo "MK: cp ${LDPATH} failed"; exit 1; }
fi

# Save the compilation options

echo "ARCHCFLAGS = ${ARCHCFLAGS}" >"${EXPORTDIR}/build/Make.defs"
echo "ARCHCXXFLAGS = ${ARCHCXXFLAGS}" >>"${EXPORTDIR}/build/Make.defs"

# Copy the NuttX include directory (retaining attributes and following symbolic links)

cp -LR -p "${TOPDIR}/include" "${EXPORTDIR}/." || \
	{ echo "MK: 'cp ${TOPDIR}/include' failed"; exit 1; }
find "${EXPORTDIR}/include" -name .svn | xargs rm -rf

# Copy the startup object file(s)

make -C ${ARCHDIR} export_head TOPDIR=${TOPDIR} EXPORT_DIR="${EXPORTDIR}"

# Copy architecture-specific header files into the arch export sub-directory.
# This is tricky because each architecture does things in a little different
# way.
#
# First copy any header files in the architecture src/ sub-directory (some
# architectures keep all of the header files there, some a few, and others
# none

cp -f "${ARCHDIR}"/*.h "${EXPORTDIR}"/arch/. 2>/dev/null

# Then look a list of possible places where other architecture-specific
# header files might be found.  If those places exist (as directories or
# as symbolic links to directories, then copy the header files from 
# those directories into the EXPORTDIR

ARCH_HDRDIRS="arm armv7-m avr avr32 board common chip mips32"
for hdir in $ARCH_HDRDIRS; do

	# Does the directory (or symbolic link) exist?

	if [ -d "${ARCHDIR}/${hdir}" -o -h "${ARCHDIR}/${hdir}" ]; then

		# Yes.. create a export sub-directory of the same name

		mkdir "${EXPORTDIR}/arch/${hdir}" || \
			{ echo "MK: 'mkdir ${EXPORTDIR}/arch/${hdir}' failed"; exit 1; }

		# Then copy the header files (only) into the new directory

		cp -f "${ARCHDIR}"/${hdir}/*.h "${EXPORTDIR}"/arch/${hdir}/. 2>/dev/null

		# One architecture has low directory called "chip" that holds the
		# header files

		if [ -d "${ARCHDIR}/${hdir}/chip" ]; then

			# Yes.. create a export sub-directory of the same name

			mkdir "${EXPORTDIR}/arch/${hdir}/chip" || \
				{ echo "MK: 'mkdir ${EXPORTDIR}/arch/${hdir}/chip' failed"; exit 1; }

			# Then copy the header files (only) into the new directory

			cp -f "${ARCHDIR}"/${hdir}/chip/*.h "${EXPORTDIR}"/arch/${hdir}/chip/. 2>/dev/null
		fi
	fi
done

# Copy OS internal header files as well.  They are used by some architecture-
# specific header files.

mkdir "${EXPORTDIR}/arch/os" || \
	{ echo "MK: 'mkdir ${EXPORTDIR}/arch/${hdir}/chip' failed"; exit 1; }
cp -f "${TOPDIR}"/sched/*.h "${EXPORTDIR}"/arch/os/. 2>/dev/null

# Add the board library to the list of libraries

if [ -f "${ARCHDIR}/board/libboard${LIBEXT}" ]; then
	LIBLIST="${LIBLIST} ${ARCHSUBDIR}/board/libboard${LIBEXT}"
fi

# Then process each library

AR=${CROSSDEV}ar
for lib in ${LIBLIST}; do
	if [ ! -f "${TOPDIR}/${lib}" ]; then
		echo "MK: Library ${TOPDIR}/${lib} does not exist"
		exit 1
	fi

	# Get some shorter names for the library

	libname=`basename ${lib} ${LIBEXT}`
	shortname=`echo ${libname} | sed -e "s/^lib//g"`

	# Copy the application library unmodified

	if [ "X${libname}" = "Xlibapps" ]; then
		cp -p "${TOPDIR}/${lib}" "${EXPORTDIR}/libs/." || \
			{ echo "MK: cp ${TOPDIR}/${lib} failed"; exit 1; }
	else

		# Create a temporary directory and extract all of the objects there
		# Hmmm.. this probably won't work if the archiver is not 'ar'

		mkdir "${EXPORTDIR}/tmp" || \
			{ echo "MK: 'mkdir ${EXPORTDIR}/tmp' failed"; exit 1; }
		cd "${EXPORTDIR}/tmp" || \
			{ echo "MK: 'cd ${EXPORTDIR}/tmp' failed"; exit 1; }
		if [ "X${WINTOOL}" = "Xy" ]; then
			WLIB=`cygpath -w "${TOPDIR}/${lib}"`
			${AR} x "${WLIB}"
		else
			${AR} x "${TOPDIR}/${lib}"
		fi

		# Rename each object file (to avoid collision when they are combined)
		# and add the file to libnuttx

		for file in `ls`; do
			mv "${file}" "${shortname}-${file}"
			if [ "X${WINTOOL}" = "Xy" ]; then
				WLIB=`cygpath -w "${EXPORTDIR}/libs/libnuttx${LIBEXT}"`
				${AR} rcs "${WLIB}" "${shortname}-${file}"
			else
				${AR} rcs "${EXPORTDIR}/libs/libnuttx${LIBEXT}" "${shortname}-${file}"
			fi
		done

		cd "${TOPDIR}" || \
			{ echo "MK: 'cd ${TOPDIR}' failed"; exit 1; }
		rm -rf "${EXPORTDIR}/tmp"
	fi
done

# Now tar up the whole export directory

cd "${TOPDIR}" || \
	{ echo "MK: 'cd ${TOPDIR}' failed"; exit 1; }

if [ "X${TGZ}" = "Xy" ]; then
	tar cvf "${EXPORTSUBDIR}.tar" "${EXPORTSUBDIR}" 1>/dev/null 2>&1
	gzip -f "${EXPORTSUBDIR}.tar"
else
	zip -r "${EXPORTSUBDIR}.zip" "${EXPORTSUBDIR}" 1>/dev/null 2>&1
fi

# Clean up after ourselves

rm -rf "${EXPORTSUBDIR}" 
