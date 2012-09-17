#!/bin/sh

# Set up pathes to binaries, scripts, configuration files

hostos=`uname -o 2>/dev/null || echo "Other"`
if [ "X${hostos}" = "XCygwin" ]; then
	installdir=/cygdrive/c/gccfd/openocd/bin
	ft2exe=$installdir/openocd-ftd2xx.exe
	ppexe=$installdir/openocd-ppdev.exe
else
	installdir=/usr/local/bin
	ft2exe=$installdir/openocd
	ppexe=$installdir/openocd
	SUDO=sudo
fi

# The root to the top-level NuttX directory should be in an environment variable

if [ -z $STR41XSCRIPTS ]; then
	echo "Environment variable $STR41XSCRIPTS is not defined"
	echo "Has NuttX been configured?"
	echo "If so, try sourcing the setenv.sh script in the top-level directory"
	exit 1
fi

# Check that at least one configuration file exists at that point
if [ ! -f $STR41XSCRIPTS/oocd_ft2xx.cfg ]; then
	echo "No configuration files found at $STR41XSCRIPTS"
	echo "Path to configuration files unknown"
	exit 1
fi

# Parse command line inputs

usage="USAGE: $0 [-h] [-d]  [-pp] [-ft2xx]"

debug=no
oocdcfg=$STR41XSCRIPTS/oocd_ft2xx.cfg
openocd=$ft2exe
while [ ! -z "$1" ]; do
	case $1 in
	-d )
		debug=yes
		set -x
		;;
	-pp )
		oocdcfg=$STR41XSCRIPTS/oocd_wiggler.cfg
		openocd=$ppexe
		;;
	-ft2xx )
		oocdcfg=$STR41XSCRIPTS/oocd_ft2xx.cfg
		openocd=$ft2exe
		;;
	-h )
		echo $usage
		exit 0
		;;
	* )
		echo "Unrecognized option: $1"
		echo $usage
		exit 1
		;;
	esac
	shift
done

# Setup debug options

export options="-d 1"

# Run OpenOCD -- here it is assumed (1) that you must have root priveleges to 
# execute OpenOCD and (2) that your user is listed in the /etc/sudoers file.

$SUDO $openocd $options -f $oocdcfg
if [ "X${hostos}" = "XCygwin" ]; then
	$openocd $options -f `cygpath -w $oocdcfg`
else
	sudo $openocd $options -f $oocdcfg
fi
