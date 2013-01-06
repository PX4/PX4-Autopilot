#!/bin/bash
############################################################################
# tools/mkdeps.sh
#
#   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
# Usage:

show_usage ()
{
  echo ""
  echo "$progname  [OPTIONS] CC -- CFLAGS -- file [file [file...]]"
  echo ""
  echo "Where:"
  echo "  CC"
  echo "    A variable number of arguments that define how to execute the compiler"
  echo "  CFLAGS"
  echo "    The compiler compilation flags"
  echo "  file"
  echo "    One or more C files whose dependencies will be checked.  Each file is expected"
  echo "    to reside in the current directory unless --dep-path is provided on the command line"
  echo ""
  echo "And [OPTIONS] include:"
  echo "  --dep-debug"
  echo "    Enable script debug"
  echo "  --dep-path <path>"
  echo "    Do not look in the current directory for the file.  Instead, look in <path> to see"
  echo "    if the file resides there.  --dep-path may be used multiple times to specify"
  echo "    multiple alternative location"
  echo "  --winpaths <TOPDIR>"
  echo "    CC generates dependency lists using Windows paths (e.g., C:\blablah\blabla).  This"
  echo "    switch instructs the script to use 'cygpath' to convert the Windows paths to Cygwin"
  echo "    paths"
  echo "  --help"
  echo "    Shows this message and exits"
  exit 1
}

dodep ()
{
  unset fullpath
  if [ -z "$altpath" ]; then
    if [ -r $1 ]; then
      fullpath=$1
    else
      echo "# ERROR: No readable file at $1"
      show_usage
    fi
  else
    for path in $altpath; do
      tmppath=$path/$1
      if [ -r $tmppath ]; then
        fullpath=$tmppath
        break;
      fi
    done
    if [ -z "$fullpath" ]; then
      echo "# ERROR: No readable file for $1 found at any location"
      show_usage
    fi
  fi

  $cc -M $cflags $fullpath || \
    ( echo "# ERROR: $cc -M $cflags $fullpath FAILED"; exit 4; )
}

unset cc
unset cflags
unset files
unset args
unset altpath
winpaths=n
unset topdir

# Accumulate CFLAGS up to "--"
progname=$0
while [ ! -z "$1" ]; do
  case $1 in
  -- )
    cc=$cflags
    cflags=$args
    args=
    ;;
  --dep-debug )
    if [ -z "$args" ]; then
      set -x
    else
      args="$args $1"
    fi
    ;;
  --dep-path )
    if [ -z "$args" ]; then
      shift
      altpath="$altpath $1"
    else
      args="$args $1"
    fi
    ;;
  --winpaths )
    if [ -z "$args" ]; then
      shift
      winpaths=y
      topdir=$1
    else
      args="$args $1"
    fi
    ;;
  --help )
    show_usage
    ;;
  *)
    args="$args $1"
    ;;
  esac
  shift
done
files=$args

if [ -z "$cc" ]; then
  echo "ERROR: No compiler specified"
  show_usage
  exit 1
fi

if [ -z "$files" ]; then
  # Don't report an error -- this happens normally in some configurations
  echo "# No files specified for dependency generataion"
  exit 0
fi

# Check if this compiler generates Cygwin/Linux paths or Windows paths

if [ "X${winpaths}" = "Xy" ]; then
    # We will have to parse and modify each dependency (yech)
    # Make sure a valid TOPDIR argument was provided

    if [ -z "$topdir" -o ! -d $topdir ]; then
    	echo "<TOPDIR> not specified or does not exist: $topdir"
    	show_usage
    	exit 1
    fi
 
    # Get the top dir expressed like the Windows GCC would use it, except
    # with forward slashs

    wtopdir=`cygpath -w ${topdir} | sed -e "s,\\\\\\,/,g"`

    # Then get the dependency and perform conversions on it to make it
    # palatable to the Cygwin make.  This is probably not sufficiently
    # general to work on all platforms (like if your disk is not C:).

    for file in $files ; do
        dodep $file | sed -e "s,\\\,/,g" -e "s,${wtopdir},${topdir},g" \
                          -e "s,/ ,\\\ ,g" -e "s,c:/,/cygdrive/c/,g" \
                          -e "s,/$,\\\,g"
    done
else
    # For normal Cygwin/Linux GCC, the dependency paths are in the
    # correct form and can simply be echoed on stdout

    for file in $files ; do
        dodep $file
    done
fi

