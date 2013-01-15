#!/bin/bash
############################################################################
# tools/mkromfsimg.sh
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

# Environmental stuff

wd=`pwd`
workingdir=$wd/img
rcsfile=rcS
rcstemplate=$rcsfile.template
romfsimg=romfs.img
headerfile=nsh_romfsimg.h

# Get the input parameters

topdir=$1
usage="USAGE: $0 <topdir>"

if [ -z "$topdir" -o ! -d "$topdir" ]; then
    echo "The full path to the NuttX base directory must be provided on the command line"
    echo $usage
    exit 1
fi

# Extract all values from the .config in the $topdir that contains all of the NuttX
# configuration settings.  The .config file was intended to be include-able by makefiles
# and source-able by scripts.  Unfortunately,there are too many syntactic differents
# to make that practical

if [ ! -r $topdir/.config ]; then
    echo "No readable file at $topdir/.config"
    echo "Has NuttX been configured?"
    exit 1
fi

romfsetc=`grep CONFIG_NSH_ROMFSETC= $topdir/.config | cut -d'=' -f2`
disablempt=`grep CONFIG_DISABLE_MOUNTPOINT= $topdir/.config | cut -d'=' -f2`
disablescript=`grep CONFIG_NSH_DISABLESCRIPT= $topdir/.config | cut -d'=' -f2`
ndescriptors=`grep CONFIG_NFILE_DESCRIPTORS= $topdir/.config | cut -d'=' -f2`
devconsole=`grep CONFIG_DEV_CONSOLE= $topdir/.config | cut -d'=' -f2`
romfs=`grep CONFIG_FS_ROMFS= $topdir/.config | cut -d'=' -f2`
romfsmpt=`grep CONFIG_NSH_ROMFSMOUNTPT= $topdir/.config | cut -d'=' -f2`
initscript=`grep CONFIG_NSH_INITSCRIPT= $topdir/.config | cut -d'=' -f2`
romfsdevno=`grep CONFIG_NSH_ROMFSDEVNO= $topdir/.config | cut -d'=' -f2`
romfssectsize=`grep CONFIG_NSH_ROMFSSECTSIZE= $topdir/.config | cut -d'=' -f2`
fatfs=`grep CONFIG_FS_FAT= $topdir/.config | cut -d'=' -f2`
fatdevno=`grep CONFIG_NSH_FATDEVNO= $topdir/.config | cut -d'=' -f2`
fatsectsize=`grep CONFIG_NSH_FATSECTSIZE= $topdir/.config | cut -d'=' -f2`
fatnsectors=`grep CONFIG_NSH_FATNSECTORS= $topdir/.config | cut -d'=' -f2`
fatmpt=`grep CONFIG_NSH_FATMOUNTPT= $topdir/.config | cut -d'=' -f2`

# The following settings are required for general ROMFS support
#
# Mountpoint support must be enabled

if [ "X$disablempt" = "Xy" ]; then
    echo "Mountpoint support is required for this feature"
    echo "Set CONFIG_DISABLE_MOUNTPOINT=n to continue"
    exit 1
fi

# Scripting support must be enabled

if [ "X$disablescript" = "Xy" ]; then
    echo "NSH scripting support is required for this feature"
    echo "Set CONFIG_NSH_DISABLESCRIPT=n to continue"
    exit 1
fi

# We need at least 2 file descriptors 1 for the ROMFS mount and one for
# FAT mount performed in rcS.  That still wouldn't be enough to to do much
# with NSH

if [ -z "$ndescriptors" -o "$ndescriptors" -lt 2 ]; then
    echo "No file descriptors have been allocated"
    if [ "X$devconsole" = "Xy" ]; then
        echo "Set CONFIG_NFILE_DESCRIPTORS to value greater than 4"
    else
        echo "Set CONFIG_NFILE_DESCRIPTORS to value greater than 1"
    fi
    exit 1
fi

# If a console is enabled, then three more file descriptors are required
# for stdin, stdout, and stderr

if [ "X$devconsole" = "Xy" -a "$ndescriptors" -lt 5 ]; then
    echo "Insufficient file descriptors have been allocated"
    echo "Set CONFIG_NFILE_DESCRIPTORS to value greater than 4"
fi

# ROMFS support is required, of course

if [ "X$romfs" != "Xy" ]; then
    echo "ROMFS support is disabled in the NuttX configuration"
    echo "Set CONFIG_FS_ROMFS=y to continue"
    exit 0
fi

# The options in the default rcS.template also require FAT FS support

if [ "X$fatfs" != "Xy" ]; then
    echo "FAT FS support is disabled in the NuttX configuration"
    echo "Set CONFIG_FS_FAT=y to continue"
    exit 0
fi

# Verify that genromfs has been installed

genromfs -h 1>/dev/null 2>&1 || { \
  echo "Host executable genromfs not available in PATH"; \
  echo "You may need to download in from http://romfs.sourceforge.net/"; \
  exit 1; \
}

# Supply defaults for all un-defined ROMFS settings

if [ -z "$romfsmpt" ]; then
    romfsmpt="/etc"
fi
if [ -z "$initscript" ]; then
    initscript="init.d/rcS"
fi
if [ -z "$romfsdevno" ]; then
    romfsdevno=0
fi
if [ -z "$romfssectsize" ]; then
    romfssectsize=64
fi

# Supply defaults for all un-defined FAT FS settings

if [ -z "$fatdevno" ]; then
    fatdevno=1
fi
if [ -z "$fatsectsize" ]; then
    fatsectsize=512
fi
if [ -z "$fatnsectors" ]; then
    fatnsectors=1024
fi
if [ -z "$fatmpt" ]; then
   fatmpt="/tmp"
fi

# Verify the mountpoint.  Verify that it is an absolute path but not /, /dev,
# /., /./*, /.., or /../*

if [ ${romfsmpt:0:1} != "\"" ]; then
   echo "CONFIG_NSH_ROMFSMOUNTPT must be a string"
   echo "Change it so that it is enclosed in quotes."
   exit 1
fi

uromfsmpt=`echo $romfsmpt | sed -e "s/\"//g"`

if [ ${uromfsmpt:0:1} != "/" ]; then
   echo "CONFIG_NSH_ROMFSMOUNTPT must be an absolute path in the target FS"
   echo "Change it so that it begins with the character '/'.  Eg. /etc"
   exit 1
fi

tmpdir=$uromfsmpt
while [ ${tmpdir:0:1} == "/" ]; do
    tmpdir=${tmpdir:1}
done

if [ -z "$tmpdir" -o "X$tmpdir" = "Xdev" -o "X$tmpdir" = "." -o \
     ${tmpdir:0:2} = "./" -o "X$tmpdir" = ".." -o ${tmpdir:0:3} = "../" ]; then
   echo "Invalid CONFIG_NSH_ROMFSMOUNTPT selection."
   exit 1
fi

# Verify that the path to the init file is a relative path and not ., ./*, .., or ../*

if [ ${initscript:0:1} != "\"" ]; then
   echo "CONFIG_NSH_INITSCRIPT must be a string"
   echo "Change it so that it is enclosed in quotes."
   exit 1
fi

uinitscript=`echo $initscript | sed -e "s/\"//g"`

if [ ${uinitscript:0:1} == "/" ]; then
   echo "CONFIG_NSH_INITSCRIPT must be an relative path in under $romfsmpt"
   echo "Change it so that it begins with the character '/'.  Eg. init.d/rcS. "
   exit 1
fi

if [ "X$uinitscript" = "."  -o ${uinitscript:0:2} = "./" -o \
     "X$uinitscript" = ".." -o ${uinitscript:0:3} = "../" ]; then
   echo "Invalid CONFIG_NSH_INITSCRIPT selection.  Must not begin with . or .."
   exit 1
fi

# Create a working directory

rm -rf $workingdir || { echo "Failed to remove the old $workingdir"; exit 1; }
mkdir -p $workingdir || { echo "Failed to created the new $workingdir"; exit 1; }

# Create the rcS file from the rcS.template

if [ ! -r $rcstemplate ]; then
    echo "$rcstemplate does not exist"
    rmdir $workingdir
    exit 1
fi

cat $rcstemplate | \
    sed -e "s,XXXMKRDMINORXXX,$fatdevno,g" | \
    sed -e "s,XXMKRDSECTORSIZEXXX,$fatsectsize,g" | \
    sed -e "s,XXMKRDBLOCKSXXX,$fatnsectors,g" | \
    sed -e "s,XXXRDMOUNTPOUNTXXX,$fatmpt,g" >$rcsfile

# And install it at the specified relative location

install -D --mode=0755 $rcsfile $workingdir/$uinitscript || \
    { echo "Failed to install $rcsfile at $workingdir/$uinitscript"; rm -f $rcsfile; exit 1; }
rm -f $rcsfile

# Now we are ready to make the ROMFS image

genromfs -f $romfsimg -d $workingdir -V "NSHInitVol" || { echo "genromfs failed" ; exit 1 ; }
rm -rf $workingdir || { echo "Failed to remove the old $workingdir"; exit 1; }

# And, finally, create the header file

xxd -i $romfsimg >$headerfile || { echo "xxd of $< failed" ; rm -f $romfsimg; exit 1 ; }
rm -f $romfsimg
