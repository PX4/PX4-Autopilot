#!/bin/bash
################################################################################
# Copyright (c) 2025 ModalAI, Inc. All rights reserved.
#
# Semi-universal script for making a deb package. This is shared
# between the vast majority of VOXL-SDK packages
#
# Add the 'timestamp' argument to add a date-timestamp suffix to the deb package
# version. This is used by CI for making nightly and development builds.
#
# author: james@modalai.com
################################################################################

set -e # exit on error to prevent bad package from being generated

################################################################################
# Check arguments
################################################################################

USETIMESTAMP=false

print_usage(){
	echo ""
	echo " Package the current project into a deb package."
	echo " You must run build.sh first to build the binaries"
	echo ""
	echo " Usage:"
	echo "  ./make_package.sh"
	echo "        Build a DEB package"
	echo ""
	echo "  ./make_package.sh timestamp"
	echo "        Build a DEB package with the current timestamp as a"
	echo "        suffix in both the package name and deb filename."
	echo "        This is used by CI for development packages."
	echo ""
	echo ""
}

process_argument () {

	if [ "$#" -ne 1 ]; then
		echo "ERROR process_argument expected 1 argument"
		exit 1
	fi

	## convert argument to lower case for robustness
	arg=$(echo "$1" | tr '[:upper:]' '[:lower:]')
	case ${arg} in
		"")
			;;
		"-t"|"timestamp"|"--timestamp")
			echo "using timestamp suffix"
			USETIMESTAMP=true
			;;
		"-d"|"deb"|"debian"|"--deb"|"--debian")
			# normal behavior, argument left for backwards compatibility
			;;
		*)
			echo "invalid option"
			print_usage
			exit 1
	esac
}


## parse all arguments or run wizard
for var in "$@"
do
	process_argument $var
done


################################################################################
# variables
################################################################################
VERSION=$(cat pkg/control/control | grep "Version" | cut -d' ' -f 2)
PACKAGE=$(cat pkg/control/control | grep "Package" | cut -d' ' -f 2)

DATA_DIR=pkg/data
CONTROL_DIR=pkg/control
DEB_DIR=pkg/DEB

echo "Package Name: " $PACKAGE
echo "version Number: " $VERSION

################################################################################
# start with a little cleanup to remove old files
################################################################################
# remove data directory where 'make install' installed to
sudo rm -rf $DATA_DIR
mkdir $DATA_DIR

# remove odl deb and packaging folders
rm -rf $DEB_DIR
rm -f *.deb


################################################################################
## install compiled stuff into data directory with 'make install'
## try this for all 3 possible build folders, some packages are multi-arch
## so both 32 and 64 need installing to pkg directory.
################################################################################

DID_BUILD=false

if [[ -d "build" ]]; then
	cd build && sudo make DESTDIR=../${DATA_DIR} PREFIX=/usr install && cd -
	DID_BUILD=true
fi
if [[ -d "build32" ]]; then
	cd build32 && sudo make DESTDIR=../${DATA_DIR} PREFIX=/usr install && cd -
	DID_BUILD=true
fi
if [[ -d "build64" ]]; then
	cd build64 && sudo make DESTDIR=../${DATA_DIR} PREFIX=/usr install && cd -
	DID_BUILD=true
fi

# make sure at least one directory worked
if [ "$DID_BUILD" = false ] && [ -f "build.sh" ]; then
	echo "neither build/ build32/ or build64/ were found"
	exit 1
fi


################################################################################
## install standard stuff common across ModalAI projects if they exist
################################################################################

if [ -d "services" ]; then
	sudo mkdir -p $DATA_DIR/etc/systemd/system/
	sudo cp services/* $DATA_DIR/etc/systemd/system/
fi

if [ -d "scripts" ]; then
	sudo mkdir -p $DATA_DIR/usr/bin/
	sudo chmod +x scripts/*
	sudo cp scripts/* $DATA_DIR/usr/bin/
fi

if [ -d "bash_completions" ]; then
	sudo mkdir -p $DATA_DIR/usr/share/bash-completion/completions
	sudo cp bash_completions/* $DATA_DIR/usr/share/bash-completion/completions
fi

if [ -d "misc_files" ]; then
	sudo cp -R misc_files/* $DATA_DIR/
fi

if [ -d "bash_profile" ]; then
	sudo mkdir -p ${DATA_DIR}/home/root/.profile.d/
	sudo cp -R bash_profile/* ${DATA_DIR}/home/root/.profile.d/
fi


################################################################################
# make a DEB package
################################################################################

echo "starting building Debian Package"

## make a folder dedicated to deb building and copy the requires debian-binary file in
mkdir $DEB_DIR

## copy the control stuff in
cp -rf $CONTROL_DIR/ $DEB_DIR/DEBIAN
cp -rf $DATA_DIR/*   $DEB_DIR

## update version with timestamp if enabled
if $USETIMESTAMP; then
	dts=$(date +"%Y%m%d%H%M")
	sed -E -i "s/Version.*/&-$dts/" $DEB_DIR/DEBIAN/control
	VERSION="${VERSION}-${dts}"
	echo "new version with timestamp: $VERSION"
fi

DEB_NAME=${PACKAGE}_${VERSION}_arm64.deb
dpkg-deb --build ${DEB_DIR} ${DEB_NAME}

echo "DONE"
