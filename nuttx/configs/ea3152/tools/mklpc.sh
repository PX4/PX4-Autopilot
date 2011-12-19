#!/bin/sh

# This script lies in sub-directory configs/ea3152/tools but make be executed
# from either that directory or TOPDIR

MYNAME=`basename $0`
if [ -x "$PWD/$MYNAME" ]; then
	TOPDIR="$PWD/../../.."
else
	if [ -x "$PWD/configs/ea3152/tools/$MYNAME" ]; then
		TOPDIR="$PWD"
	else
		echo "This script must be executed from a known director"
		exit 1
	fi
fi
echo "TOOLDIR: $TOOLDIR"

# The lpchdr could be named lpchdr.exe if we are running under Cygwin or
# just lpchdr under Linux

TOOLDIR=$TOPDIR/configs/ea3152/tools

if [ ! -d "$TOOLDIR" ]; then
	echo "Tool directory $TOOLDIR does not exist"
	exit 1
fi

if [ -x "$TOOLDIR/lpchdr.exe" ]; then
	LPCHDR="$TOOLDIR/lpchdr.exe"
else
	if  [ -x "$TOOLDIR/lpchdr" ]; then
		LPCHDR="$TOOLDIR/lpchdr"
	else
		echo "lpchdr executable does not exist in $TOODIR"
		echo " - cd $TOOLDIR"
		echo " - make"
	fi
fi
echo "LPCHDR: $LPCHDR"

# Now get the path to the NuttX executable

NUTTXPATH="$TOPDIR/nuttx.bin"

if [ ! -f "$NUTTXPATH" ]; then
	echo "NuttX binary does not exist at $NUTTXPATH"
	echo " - cd $TOPDIR"
	echo " - make"
	exit 1
fi
echo "NUTTXPATH: $NUTTXPATH"

# Create the binary

echo "COMMAND: $LPCHDR -o $TOPDIR/nuttx.lpc $NUTTXPATH"
"$LPCHDR" -o "$TOPDIR/nuttx.lpc" "$NUTTXPATH" || \
	{ echo "$LPCHDR failed" ; exit 1 ; }
echo "Successfully created binary"

