#! /usr/bin/env bash

set -e

## Bash script to setup PX4 development environment on Ubuntu LTS (22.04, 20.04, 18.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for NuttX, Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## Optional:
## - Gazebo simulator (build with arg: --with-sim-tools)
## - Run within Docker (build with --from-docker)
##

INSTALL_NUTTX="true"
INSTALL_SIM="false"
INSTALL_GAZEBO_CLASSIC="false"
INSTALL_GAZEBO="false"
INSTALL_ARCH=$(uname -m)
INSIDE_DOCKER="false"

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--no-nuttx" ]]; then
		INSTALL_NUTTX="false"
	fi

	# we need to keep this to retain backwards compatibility
	# with older scripts using ubuntu.sh
	if [[ $arg == "--no-sim-tools" ]]; then
		INSTALL_SIM="false"
	fi

	if [[ $arg == "--with-sim-tools" ]]; then
		INSTALL_SIM="true"
	fi

	if [[ $arg == "--from-docker" ]]; then
		INSIDE_DOCKER="true"
	fi

	if [[ $arg == "--help" ]]; then
		usage
		exit 0
	fi

done

# Script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Check requirements.txt exists (script not run in source tree)
REQUIREMENTS_FILE="requirements.txt"
if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
	echo "Failed: ${REQUIREMENTS_FILE} needed in same directory as ubuntu.sh (${DIR})."
	exit 1
fi


# Check ubuntu version
UBUNTU_RELEASE=$(lsb_release -rs)

if [[ "${UBUNTU_RELEASE}" == "14.04" ]]; then
	echo "Ubuntu 14.04 is no longer supported"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
	echo "Ubuntu 16.04 is no longer supported"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
	echo "Ubuntu 18.04"
elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
	echo "Ubuntu 20.04"
elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
	echo "Ubuntu 22.04"
fi

VERBOSE_BAR="================================================================================"
echo
echo $VERBOSE_BAR
echo "⚡️ Starting PX4 Dependency Installer for Ubuntu ${UBUNTU_RELEASE} (${INSTALL_ARCH})"
echo ""
echo "Options:
- Install NuttX toolchain: ${INSTALL_NUTTX}
- Install Simuation:  ${INSTALL_SIM}"
echo $VERBOSE_BAR
echo

echo
echo $VERBOSE_BAR
echo "🍻 Installing System Dependencies"
echo $VERBOSE_BAR
echo

sudo apt-get update -y --quiet
sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
	astyle \
	build-essential \
	cmake \
	cppcheck \
	file \
	g++ \
	gcc \
	gdb \
	git \
	gnupg \
	lcov \
	libfuse2 \
	libxml2-dev \
	libxml2-utils \
	make \
	ninja-build \
	python3 \
	python3-dev \
	python3-pip \
	python3-setuptools \
	python3-wheel \
	rsync \
	shellcheck \
	unzip \
	wget \
	zip \
	libssl-dev \
	;

# Python 3 dependencies
echo
echo $VERBOSE_BAR
echo "🍻 Installing Python dependencies"
echo $VERBOSE_BAR
echo

python3 -m pip install -r "$DIR"/requirements.txt

# NuttX toolchain (arm-none-eabi-gcc)
if [[ $INSTALL_NUTTX == "true" ]]; then

	echo
	echo $VERBOSE_BAR
	echo "🍻 Installing NuttX dependencies"
	echo $VERBOSE_BAR
	echo

	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		automake \
		binutils-dev \
		bison \
		flex \
		genromfs \
		gettext \
		gperf \
		libelf-dev \
		libexpat-dev \
		libgmp-dev \
		libisl-dev \
		libmpc-dev \
		libmpfr-dev \
		libncurses5 \
		libncurses5-dev \
		libncursesw5-dev \
		libtool \
		pkg-config \
		screen \
		texinfo \
		u-boot-tools \
		util-linux \
		g++-arm-linux-gnueabihf \
		gcc-arm-linux-gnueabihf \
		g++-aarch64-linux-gnu \
		gcc-aarch64-linux-gnu \
		;
	if [[ "${UBUNTU_RELEASE}" == "20.04" || "${UBUNTU_RELEASE}" == "22.04" ]]; then
		sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		kconfig-frontends \
		;
	fi

	if [ -n "$USER" ]; then
		# Add user to dialout group (serial port access)
		sudo usermod -a -G dialout "$USER"
	fi

	NUTTX_GCC_VERSION="10.3-2021.10"
	echo
	echo $VERBOSE_BAR
	echo "🍻 Verifying arm-none-eabi-gcc version (${NUTTX_GCC_VERSION}), and installing if not found"
	echo

	source "$HOME/.profile" # load changed path for the case the script is reran before relogin
	if [ "$(which arm-none-eabi-gcc)" ]; then
		GCC_VER_STR=$(arm-none-eabi-gcc --version)
	fi

	if [[ $(echo "$GCC_VER_STR" | grep -c "${NUTTX_GCC_VERSION}") == "1" ]]; then
		echo "📌 Skipping installation, the arm cross compiler was found"
		echo $VERBOSE_BAR
		echo

	else
		echo "📌 The arm cross compiler was not found";
		echo " * Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		# The arm cross compiler hosting provider is known to throttle download speeds
		# for users who reach a certain limit of downloads in a given time frame
		# for this reason we allow for using a previously downloaded file
		# this is specially helpful when debugging this installer script
		# from within a container COMPILER_PATH="/tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2"
		COMPILER_NAME="gcc-arm-none-eabi-${NUTTX_GCC_VERSION}"
		COMPILER_PATH="/tmp/$COMPILER_NAME-linux.tar.bz2"
		if [ ! -f "$COMPILER_PATH" ]; then
		wget -O "/tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2" "https://developer.arm.com/-/media/Files/downloads/gnu-rm/${NUTTX_GCC_VERSION}/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-${INSTALL_ARCH}-linux.tar.bz2"
		fi
		sudo tar -jxf $COMPILER_PATH -C /opt/;

		# add arm-none-eabi-gcc to user's PATH
		exportline="export PATH=\"/opt/${COMPILER_NAME}/bin:\$PATH\""
		if [[ $INSIDE_DOCKER == "true" ]]; then
			# when running on a docker container its best to set the environment globally
			# since we don't know which user is going to be running commands on the container
			touch /etc/profile.d/px4env.sh
			echo "$exportline" >> /etc/profile.d/px4env.sh
		elif grep -Fxq "$exportline" "$HOME"/.profile; then
			echo "${NUTTX_GCC_VERSION} path already set.";
		else
			echo $exportline >> $HOME/.profile;
			source $HOME/.profile; # Allows to directly build NuttX targets in the same terminal
		fi
		echo " * arm-none-eabi-gcc (${NUTTX_GCC_VERSION}) Installed successfully to /opt/${COMPILER_NAME}/bin"
		echo $VERBOSE_BAR
		echo
	fi
fi

install_gazebo_common() {
	# General simulation dependencies
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		bc \
		;

	# Installing Gazebo and dependencies
	# Setup OSRF Gazebo repository
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	# Update list, since new gazebo-stable.list has been added
	sudo apt-get update -y --quiet

	# Install Sim Dev Dependencies
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		dmidecode \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		libeigen3-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		pkg-config \
		protobuf-compiler \
		;

	if sudo dmidecode -t system | grep -q "Manufacturer: VMware, Inc." ; then
		# fix VMWare 3D graphics acceleration for gazebo
		echo "export SVGA_VGPU10=0" >> ~/.profile
	fi
}

# decide which sim to install if any
# then install common depdendencies
if [[ $INSTALL_SIM == "true" ]]; then
	install_gazebo_common

	if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
		INSTALL_GAZEBO_CLASSIC = "true"
	elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
		INSTALL_GAZEBO = "true"
	fi
fi

# Gazebo Classic
if [[ $INSTALL_GAZEBO_CLASSIC == "true" ]]; then

	echo
	echo $VERBOSE_BAR
	echo "🍻 Installing Gazebo Classic"
	echo
	echo "  * Gazebo Classic (Version 11)"
	echo $VERBOSE_BAR

	# Install Gazebo classic
	if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
		# Install Gazebo Citadel
		gazebo_classic_version=9
		gazebo_packages="gazebo$gazebo_classic_version libgazebo$gazebo_classic_version-dev"
	else
		# Install Gazebo Fortress
		gazebo_classic_version=11
		gazebo_packages="gazebo$gazebo_classic_version libgazebo$gazebo_classic_version-dev"
	fi


	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		dmidecode \
		$gazebo_packages \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		libeigen3-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		pkg-config \
		protobuf-compiler \
		;
fi

# New Gazebo
if [[ $INSTALL_GAZEBO == "true" ]]; then

	echo
	echo $VERBOSE_BAR
	echo "🍻 Installing Gazebo"
	echo
	echo "  * Gazebo Garden"
	echo $VERBOSE_BAR

	install_gazebo_common

	# Gazebo installation

	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		gz-garden \
		;
fi


if [[ $INSIDE_DOCKER == "true" ]]; then
	# cleanup installation
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
fi

if [[ $INSIDE_DOCKER == "false" ]] && [[ $INSTALL_NUTTX == "true" ]]; then
	echo
	echo $VERBOSE_BAR
	echo "💡 We recommend you relogin/reboot before attempting to upload NuttX targets"
	echo "   to be part of the dialout group to have access to serial ports."
	echo $VERBOSE_BAR
	echo
fi

echo
echo
echo $VERBOSE_BAR
echo "⚡️ PX4 Dependency Installer ended successfully

For more information on PX4 Autopilot check out our docs
at https://docs.px4.io.
If you find a bug please file an issue
on https://github.com/PX4/PX4-Autopilot"
echo $VERBOSE_BAR
echo
