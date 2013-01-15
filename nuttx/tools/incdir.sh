#!/bin/bash
# tools/incdir.sh
#
#   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

# Handle command line options

progname=$0
wintool=n
pathtype=user
usage="USAGE: $progname [-w] [-d] [-h] <compiler-path> <dir1> [<dir2> [<dir3> ...]]"
advice="Try '$progname -h' for more information"

while [ ! -z "$1" ]; do 
	case $1 in
	-d )
		set -x
		;;
	-w )
		wintool=y
		;;
	-s )
		pathtype=system
		;;
	-h )
		echo "$progname is a tool for flexible generation of include path arguments for a"
		echo "variety of different compilers in a variety of compilation environments"
		echo ""
		echo $usage
		echo ""
		echo "Where:"
		echo "	<compiler-path>"
		echo "		The full path to your compiler"
		echo "	<dir1> [<dir2> [<dir3> ...]]"
		echo "		A list of include directories"
		echo "	-w"
		echo "		The compiler is a Windows native tool and requires Windows"
		echo "		style pathnames like C:\\Program Files"
		echo "	-s"
		echo "		Generate standard, system header file paths instead of normal user"
		echo "		header file paths."
		echo "	-d"
		echo "		Enable script debug"
		echo "	-h"
		echo "		Shows this help text and exits."
		exit 0
		;;
	* )
		break;
		;;
	esac
	shift
done

ccpath=$1
shift
dirlist=$@

if [ -z "$ccpath" ]; then
	echo "Missing compiler path"
	echo $usage
	echo $advice
	exit 1
fi

if [ -z "$dirlist" ]; then
	echo "Missing include directory list"
	echo $usage
	echo $advice
	exit 1
fi

#
# Most compilers support CFLAG options like '-I<dir>' to add include
# file header paths.  Some (like the Zilog tools), do not.  This script
# makes the selection of header file paths compiler independent.
#
# Below are all known compiler names (as found in the config/*/*/Make.defs
# files).  If a new compiler is used that has some unusual syntax, then
# additional logic needs to be added to this file.
#
#   NAME                        Syntax
#   $(CROSSDEV)gcc              -I<dir1> -I<dir2> -I<dir3> ...
#   sdcc                        -I<dir2> -I<dir2> -I<dir3> ...
#   $(ZDSBINDIR)/ez8cc.exe      -usrinc:'<dir1>:<dir2>:<dir3>:...`
#   $(ZDSBINDIR)/zneocc.exe     -usrinc:'<dir1>:<dir2>:<dir3>:...`
#   $(ZDSBINDIR)/ez80cc.exe     -usrinc:'<dir1>:<dir2>:<dir3>:...`
#
# Furthermore, just to make matters more difficult, with Windows based
# toolchains, we have to use the full windows-style paths to the header
# files.

os=`uname -o 2>/dev/null || echo "Other"`

# Let's assume that all GCC compiler paths contain the string gcc or
# g++ and no non-GCC compiler pathes include these substrings

gcc=`echo $ccpath | grep gcc`
if [ -z "${gcc}" ]; then
  gcc=`echo $ccpath | grep g++`
fi

sdcc=`echo $ccpath | grep sdcc`

if [ "X$os" = "XCygwin" ]; then
	# We can treat Cygwin native toolchains just like Linux native
	# toolchains in the Linux.  Let's assume:
	# 1. GCC or SDCC are the only possible Cygwin native compilers
	# 2. If this is a Window native GCC version, then -w must be
	#    provided on the command line (wintool=y)

	if [ -z "$gcc" -a -z "$sdcc" ]; then
		# Not GCC or SDCC, must be Windows native
		windows=yes
		compiler=`cygpath -u "$ccpath"`
	else
		if [ "X$wintool" == "Xy" ]; then
			# It is a native GCC or SDCC compiler
			windows=yes
			compiler=`cygpath -u "$ccpath"`
		else
			# GCC or SDCC and not for Windows
			windows=no
			compiler="$ccpath"
		fi
	fi
else
	# Otherwise, we must be in a Linux environment where there are
	# only Linux native toolchains
	windows=no
	compiler="$ccpath"
fi
exefile=`basename "$compiler"`

# Check for some well known, non-GCC Windows native tools that require
# a special output format as well as special paths

if [ "X$exefile" = "Xez8cc.exe" -o "X$exefile" = "Xzneocc.exe" -o "X$exefile" = "Xez80cc.exe" ]; then
	fmt=zds
else
	fmt=std
fi

# Select system or user header file path command line option

if [ "X$fmt" = "Xzds" ]; then
	if [ "X$pathtype" = "Xsystem" ]; then
		cmdarg=-stdinc:
	else
		cmdarg=-usrinc:
	fi
else
	if [ "X$pathtype" = "Xsystem" ]; then
		cmdarg=-isystem
	else
		cmdarg=-I
	fi
fi

# Now process each directory in the directory list

unset response
for dir in $dirlist; do

	# Verify that the include directory exists

	if [ ! -d $dir ]; then
		echo "Include path '$dir' does not exist"
		echo $showusage
		exit 1
	fi

	# Check if the path needs to be extended for Windows-based tools under Cygwin

	if [ "X$windows" = "Xyes" ]; then
		path=`cygpath -w $dir`
	else
		path=$dir
	fi

	# Handle the output using the selected format

	if [ "X$fmt" = "Xzds" ]; then
		# Treat the first directory differently

		if [ -z "$response" ]; then
			response="${cmdarg}'"${path}
		else
			response=${response}";${path}"
		fi
	else
		# Treat the first directory differently

		if [ -z "$response" ]; then
			response="${cmdarg} \"$path\""
		else
			response="${response} ${cmdarg} \"$path\""
		fi
	fi
done

if [ "X$fmt" = "Xzds" ]; then
	response=$response"'"
fi

echo $response


