#!/bin/sh
# vi: set sw=4 ts=4:
# set -x

echo ""
echo "Checking build system dependencies:"

#############################################################
#
# check build system 'environment'
#
#############################################################

if test -n "$CC" ; then
	echo "CC clean:						FALSE"
	echo ""
	echo ""
	echo "You must run 'unset CC' so buildroot can run with"
	echo "a clean environment on your build machine"
	echo ""
	exit 1
fi
echo "CC clean:					Ok"

if test -n "$CXX" ; then
	echo "CXX clean:					FALSE"
	echo ""
	echo ""
	echo "You must run 'unset CXX' so buildroot can run with"
	echo "a clean environment on your build machine"
	echo ""
	exit 1
fi
echo "CXX clean:					Ok"

if test -n "$CPP" ; then
	echo "CPP clean:					FALSE"
	echo ""
	echo ""
	echo "You must run 'unset CPP' so buildroot can run with"
	echo "a clean environment on your build machine"
	echo ""
	exit 1
fi
echo "CPP clean:					Ok"

if test -n "$CFLAGS" ; then
	echo "CFLAGS clean:					FALSE"
	echo ""
	echo ""
	echo "You must run 'unset CFLAGS' so buildroot can run with"
	echo "a clean environment on your build machine"
	echo ""
	exit 1
fi
echo "CFLAGS clean:					Ok"

if test -n "$INCLUDES" ; then
	echo "INCLUDES clean:					FALSE"
	echo "WARNING: INCLUDES contains:"
	echo "        '$INCLUDES'"
else
	echo "INCLUDES clean:					Ok"
fi

if test -n "$CXXFLAGS" ; then
	echo "CXXFLAGS clean:					FALSE"
	echo ""
	echo ""
	echo "You must run 'unset CXXFLAGS' so buildroot can run with"
	echo "a clean environment on your build machine"
	echo ""
	exit 1
fi
echo "CXXFLAGS clean:					Ok"

echo "WORKS" | grep "WORKS" >/dev/null 2>&1
if test $? != 0 ; then
	echo "grep works:				FALSE"
	exit 1
fi

# sanity check for CWD in LD_LIBRARY_PATH
# try not to rely on egrep..
if test -n "$LD_LIBRARY_PATH" ; then
	echo TRiGGER_start"$LD_LIBRARY_PATH"TRiGGER_end | grep ':.:' >/dev/null 2>&1 ||
	echo TRiGGER_start"$LD_LIBRARY_PATH"TRiGGER_end | grep 'TRiGGER_start:' >/dev/null 2>&1 ||
	echo TRiGGER_start"$LD_LIBRARY_PATH"TRiGGER_end | grep ':TRiGGER_end' >/dev/null 2>&1 ||
	echo TRiGGER_start"$LD_LIBRARY_PATH"TRiGGER_end | grep '::' >/dev/null 2>&1
	if test $? = 0; then
		echo "LD_LIBRARY_PATH sane:				FALSE"
		echo "You seem to have the current working directory in your"
		echo "LD_LIBRARY_PATH environment variable. This doesn't work."
		exit 1;
	else
		echo "LD_LIBRARY_PATH sane:				Ok"
	fi
fi

#############################################################
#
# check build system 'sed'
#
#############################################################

if test -x /usr/bin/sed ; then
	SED="/usr/bin/sed"
else
	if test -x /bin/sed ; then
		SED="/bin/sed"
	else
		SED="sed"
	fi
fi
echo "HELLO" > .sedtest
$SED -i -e "s/HELLO/GOODBYE/" .sedtest >/dev/null 2>&1
if test $? != 0 ; then
	echo "sed works:				No"
	exit 1
else
	echo "sed works:					Ok"
fi
rm -f .sedtest

#############################################################
#
# check build system 'which'
#
#############################################################

if ! which which > /dev/null ; then
	echo "which installed:			FALSE"
	echo ""
	echo ""
	echo "You must install 'which' on your build machine"
	echo ""
	exit 1
fi
echo "which installed:				Ok"

#############################################################
#
# check build system 'make'
#
#############################################################

MAKE=$(which make)
if [ -z "$MAKE" ] ; then
	echo "make installed:			FALSE"
	echo ""
	echo ""
	echo "You must install 'make' on your build machine"
	echo ""
	exit 1
fi

MAKE_VERSION=$($MAKE --version 2>&1 | head -n1 | $SED -e 's/^.* \([0-9\.]\)/\1/g' -e 's/[-\ ].*//g')
if [ -z "$MAKE_VERSION" ] ; then
	echo "make installed:			FALSE"
	echo ""
	echo ""
	echo "You must install 'make' on your build machine"
	echo ""
	exit 1
fi

MAKE_MAJOR=$(echo $MAKE_VERSION | $SED -e "s/\..*//g")
MAKE_MINOR=$(echo $MAKE_VERSION | $SED -e "s/^$MAKE_MAJOR\.//g" -e "s/\..*//g" -e "s/[a-zA-Z].*//g")
if [ $MAKE_MAJOR -lt 3 -o $MAKE_MAJOR -eq 3 -a $MAKE_MINOR -lt 8 ] ; then
	echo "You have make '$MAKE_VERSION' installed.  GNU make >=3.80 is required"
	exit 1;
fi
echo "GNU make version '$MAKE_VERSION':			Ok"

#############################################################
#
# check build system 'gcc'
#
#############################################################

COMPILER=$(which $HOSTCC)
if [ -z "$COMPILER" ] ; then
	COMPILER=$(which cc)
fi

if [ -z "$COMPILER" ] ; then
	echo "C Compiler installed:			FALSE"
	echo ""
	echo ""
	echo "You must install 'gcc' on your build machine"
	echo ""
	exit 1
fi

COMPILER_VERSION=$($COMPILER -v 2>&1 | $SED -n '/^gcc version/p' |
	$SED -e 's/^gcc version \([0-9\.]\)/\1/g' -e 's/[-\ ].*//g' -e '1q')
if [ -z "$COMPILER_VERSION" ] ; then
	echo "gcc installed:			FALSE"
	echo ""
	echo ""
	echo "You must install 'gcc' on your build machine"
	echo ""
	exit 1
fi

COMPILER_MAJOR=$(echo $COMPILER_VERSION | $SED -e "s/\..*//g")
COMPILER_MINOR=$(echo $COMPILER_VERSION | $SED -e "s/^$COMPILER_MAJOR\.//g" -e "s/\..*//g")
if [ $COMPILER_MAJOR -lt 3 -o $COMPILER_MAJOR -eq 2 -a $COMPILER_MINOR -lt 95 ] ; then
	echo "You have gcc '$COMPILER_VERSION' installed.  gcc >= 2.95 is required"
	exit 1;
fi
echo "C compiler '$COMPILER'"
echo "C compiler version '$COMPILER_VERSION':			Ok"

# check for host CXX
CXXCOMPILER=$(which $HOSTCXX 2>/dev/null)
if [ -z "$CXXCOMPILER" ] ; then
	CXXCOMPILER=$(which c++ 2>/dev/null)
fi

if [ -z "$CXXCOMPILER" ] ; then
	echo "C++ Compiler installed:			FALSE"
	echo ""
	echo "You may have to install 'g++' on your build machine"
	echo ""
	#exit 1
fi

if [ ! -z "$CXXCOMPILER" ] ; then
	CXXCOMPILER_VERSION=$($CXXCOMPILER -v 2>&1 | $SED -n '/^gcc version/p' |
		$SED -e 's/^gcc version \([0-9\.]\)/\1/g' -e 's/[-\ ].*//g' -e '1q')
	if [ -z "$CXXCOMPILER_VERSION" ] ; then
		echo "c++ installed:			FALSE"
		echo ""
		echo "You may have to install 'g++' on your build machine"
		echo ""
		#exit 1
	fi

	CXXCOMPILER_MAJOR=$(echo $CXXCOMPILER_VERSION | $SED -e "s/\..*//g")
	CXXCOMPILER_MINOR=$(echo $CXXCOMPILER_VERSION | $SED -e "s/^$CXXCOMPILER_MAJOR\.//g" -e "s/\..*//g")
	if [ $CXXCOMPILER_MAJOR -lt 3 -o $CXXCOMPILER_MAJOR -eq 2 -a $CXXCOMPILER_MINOR -lt 95 ] ; then
		echo "You have g++ '$CXXCOMPILER_VERSION' installed.  g++ >= 2.95 is required"
		exit 1
	fi
	echo "C++ compiler '$CXXCOMPILER'"
	echo "C++ compiler version '$CXXCOMPILER_VERSION':			Ok"
fi

#############################################################
#
# check build system 'bison'
#
#############################################################

if ! which bison > /dev/null ; then
	echo "bison installed:			FALSE"
	echo ""
	echo ""
	echo "You must install 'bison' on your build machine"
	echo ""
	exit 1
fi
echo "bison installed:				Ok"

#############################################################
#
# check build system 'flex'
#
#############################################################

if ! which flex > /dev/null ; then
	echo "flex installed:			FALSE"
	echo ""
	echo ""
	echo "You must install 'flex' on your build machine"
	echo ""
	exit 1
fi
echo "flex installed:					Ok"

#############################################################
#
# check build system 'gettext'
#
#############################################################

if ! which msgfmt > /dev/null ; then
	echo "gettext installed:			FALSE"
	echo ""
	echo ""
	echo "You must install 'gettext' on your build machine"
	echo ""
	exit 1
fi
echo "gettext installed:				Ok"

if ! which makeinfo > /dev/null ; then
	echo "makeinfo installed:			FALSE"
	echo ""
	echo ""
	echo "Most likely some packages will fail to build their documentation"
	echo "Either install 'makeinfo' on your host or fix the respective packages"
else
	echo "makeinfo installed:				Ok"
fi

#############################################################
#
# All done
#
#############################################################

echo "Build system dependencies:			Ok"
echo ""

