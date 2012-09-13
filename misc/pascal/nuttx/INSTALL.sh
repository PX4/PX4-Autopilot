############################################################################
# nuttx/INSTALL.sh
# Install the pascaldirl runtime into the NuttX source tree
#
#   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
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

# Parse command arguments

wd=`pwd`

modeldir=insn16
unset installdir
while [ ! -z "$1" ]; do
  case "$1" in
    -d )
      set -x
      ;;
    -16 )
      modeldir=insn16
      ;;
    -32 )
      modeldir=insn32
      ;;
    -h )
      echo "USAGE: $0 [-16|-32] <install-dir>"
      exit 0
      ;;
    *)
      installdir=$1
      ;;
  esac
  shift
done

echo "Installing model $modeldir to $installdir"

# Verify that required parameters were provided

if [ -z "${installdir}" ]; then
  echo "USAGE: $0 [-16|-32]  <install-dir>"
  exit 1
fi

# Find the directory we were executed from and that things look sane

myname=`basename $0`

if [ -x ${wd}/${myname} ] ; then
   pascaldir=`dirname ${wd}`
else
   if [ -x ${wd}/nuttx/${myname} ] ; then
     pascaldir=${wd}
   else
     echo "You must cd into the pascal directory to execute this script."
     exit 1
   fi
fi

if [ ! -d ${pascaldir}/${modeldir} ]; then
  echo "Subdirectory ${modeldir} does not exist"
  exit 1
fi

if [ ! -d ${installdir} ]; then
  echo "NuttX apps/ sub-directory ${installdir} does not exist"
  exit 1
fi

if [ -d ${installdir}/pcode ]; then
  echo "${installdir}/pcode already exists.  Remove it and try again."
  exit 1
fi

# Looks good enough.  Create NuttX directories

mkdir ${installdir}/pcode || \
  { echo "mkdir ${installdir}/pcode failed" ; exit 1 ; }

mkdir ${installdir}/pcode/include || \
  { echo "mkdir ${installdir}/pcode/include failed" ; exit 1 ; }

mkdir ${installdir}/pcode/insn || \
  { echo "mkdir ${installdir}/pcode/insn failed" ; exit 1 ; }

mkdir ${installdir}/pcode/insn/include || \
  { echo "mkdir ${installdir}/pcode/insn/include failed" ; exit 1 ; }

mkdir ${installdir}/pcode/insn/prun || \
  { echo "mkdir ${installdir}/pcode/insn/prun failed" ; exit 1 ; }

mkdir ${installdir}/pcode/libpoff || \
  { echo "mkdir ${installdir}/pcode/libpoff failed" ; exit 1 ; }

mkdir ${installdir}/pcode/libpas || \
  { echo "mkdir ${installdir}/pcode/libpas failed" ; exit 1 ; }

# Copy runtime files

cp -a ${pascaldir}/include/poff.h   ${pascaldir}/include/pofflib.h \
      ${pascaldir}/include/pedefs.h ${pascaldir}/include/perr.h \
      ${pascaldir}/include/pdefs.h  ${pascaldir}/include/pfdefs.h \
      ${pascaldir}/include/pxdefs.h ${pascaldir}/include/paslib.h \
      ${installdir}/pcode/include/. || \
  { echo "Failed to copy ${pascaldir}/include" ; exit 1; }

echo "#ifndef __CONFIG_H" >${installdir}/pcode/include/config.h
echo "#define __CONFIG_H 1" >>${installdir}/pcode/include/config.h
echo "" >>${installdir}/pcode/include/config.h
echo "#undef  CONFIG_DEBUG" >>${installdir}/pcode/include/config.h
echo "#undef  CONFIG_TRACE" >>${installdir}/pcode/include/config.h
echo "#define CONFIG_INSN16 1" >>${installdir}/pcode/include/config.h
echo "#undef  CONFIG_INSN32" >>${installdir}/pcode/include/config.h
echo "" >>${installdir}/pcode/include/config.h
echo "#endif /* __CONFIG_H */" >>${installdir}/pcode/include/config.h

cp -a ${pascaldir}/nuttx/Makefile ${installdir}/pcode/. || \
  { echo "Failed to copy ${pascaldir}/nuttx/Makefile" ; exit 1; }

cp -a ${pascaldir}/nuttx/keywords.h ${installdir}/pcode/include/. || \
  { echo "Failed to copy ${pascaldir}/nuttx/keywords.h" ; exit 1; }

cp -a ${pascaldir}/libpoff/*.c ${pascaldir}/libpoff/*.h \
      ${pascaldir}/libpoff/Make.defs ${installdir}/pcode/libpoff/. || \
  { echo "Failed to copy ${pascaldir}/libpoff" ; exit 1; }

cp -a ${pascaldir}/libpas/psignextend16.c ${pascaldir}/libpas/pswap.c \
      ${pascaldir}/libpas/Make.defs ${installdir}/pcode/libpas/. || \
  { echo "Failed to copy ${pascaldir}/libpas" ; exit 1; }

cp -a ${pascaldir}/${modeldir}/include/pexec.h  ${pascaldir}/${modeldir}/include/pinsn16.h \
      ${installdir}/pcode/insn/include/. || \
  { echo "Failed to copy ${pascaldir}/${modeldir}/include" ; exit 1; }

cp -a ${pascaldir}/${modeldir}/prun/pexec.c   ${pascaldir}/${modeldir}/prun/pload.c \
      ${pascaldir}/${modeldir}/prun/Make.defs ${installdir}/pcode/insn/prun/. || \
  { echo "Failed to copy ${pascaldir}/${modeldir}/prun" ; exit 1; }
