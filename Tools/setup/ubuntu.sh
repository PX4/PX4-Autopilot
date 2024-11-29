#! /usr/bin/env bash

set -e

## Bash script to setup PX4 development environment on Ubuntu LTS (24.04, 22.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for nuttx, jMAVSim, Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## - jMAVSim and Gazebo9 simulator (omit with arg: --no-sim-tools)
##

INSTALL_NUTTX="true"
INSTALL_SIM="true"
INSTALL_ARCH=`uname -m`

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--no-nuttx" ]]; then
		INSTALL_NUTTX="false"
	fi

	if [[ $arg == "--no-sim-tools" ]]; then
		INSTALL_SIM="false"
	fi
done

# detect if running in docker
if [ -f /.dockerenv ]; then
	echo "Running within docker, installing initial dependencies";
	apt-get --quiet -y update && DEBIAN_FRONTEND=noninteractive apt-get --quiet -y install \
		ca-certificates \
		gnupg \
		gosu \
		lsb-release \
		software-properties-common \
		sudo \
		wget \
		;
fi

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# check requirements.txt exists (script not run in source tree)
REQUIREMENTS_FILE="requirements.txt"
if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
	echo "FAILED: ${REQUIREMENTS_FILE} needed in same directory as ubuntu.sh (${DIR})."
	return 1
fi


# check ubuntu version
# otherwise warn and point to docker?
UBUNTU_RELEASE="`lsb_release -rs`"
echo "Ubuntu ${UBUNTU_RELEASE}"

echo
echo "Installing PX4 general dependencies"

sudo apt-get update -y --quiet
sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
	astyle \
	build-essential \
	ccache \
	cmake \
	cppcheck \
	file \
	g++ \
	gcc \
	gdb \
	git \
	lcov \
	libssl-dev \
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
	zip \
	;

# Python3 dependencies
echo
echo "Installing PX4 Python3 dependencies"
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
REQUIRED_VERSION="3.11"
if [[ "$(printf '%s\n' "$REQUIRED_VERSION" "$PYTHON_VERSION" | sort -V | head -n1)" == "$REQUIRED_VERSION" ]]; then
	python3 -m pip install --break-system-packages -r ${DIR}/requirements.txt
else
	if [ -n "$VIRTUAL_ENV" ]; then
		# virtual environments don't allow --user option
		python -m pip install -r ${DIR}/requirements.txt
	else
		python3 -m pip install --user -r ${DIR}/requirements.txt
	fi
fi

# NuttX toolchain (arm-none-eabi-gcc)
if [[ $INSTALL_NUTTX == "true" ]]; then

	echo
	echo "Installing NuttX dependencies"

	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		automake \
		binutils-dev \
		bison \
		build-essential \
		flex \
		g++-multilib \
		gcc-arm-none-eabi \
		gcc-multilib \
		gdb-multiarch \
		genromfs \
		gettext \
		gperf \
		kconfig-frontends \
		libelf-dev \
		libexpat-dev \
		libgmp-dev \
		libisl-dev \
		libmpc-dev \
		libmpfr-dev \
		libncurses-dev \
		libncurses6 \
		libncursesw6 \
		libnewlib-arm-none-eabi \
		libstdc++-arm-none-eabi-newlib \
		libtool \
		libunwind-dev \
		pkg-config \
		screen \
		texinfo \
		u-boot-tools \
		util-linux \
		vim-common \
		;

	if [ -n "$USER" ]; then
		# add user to dialout group (serial port access)
		sudo usermod -aG dialout $USER
	fi
fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then

	echo
	echo "Installing PX4 simulation dependencies"

	# General simulation dependencies
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		bc \
		;

	# Gazebo / Gazebo classic installation
	if [[ "${UBUNTU_RELEASE}" == "18.04" || "${UBUNTU_RELEASE}" == "20.04" ]]; then
		sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
		wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
		# Update list, since new gazebo-stable.list has been added
		sudo apt-get update -y --quiet

		# Install Gazebo classic
		if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
			gazebo_classic_version=9
			gazebo_packages="gazebo$gazebo_classic_version libgazebo$gazebo_classic_version-dev"
		else
			# default and Ubuntu 20.04
			gazebo_classic_version=11
			gazebo_packages="gazebo$gazebo_classic_version libgazebo$gazebo_classic_version-dev"
		fi
	elif [[ "${UBUNTU_RELEASE}" == "21.3" ]]; then
		echo "Gazebo (Garden) will be installed"
		echo "Earlier versions will be removed"
		# Add Gazebo binary repository
		sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
		echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

		sudo apt-get update -y --quiet

		# Install Gazebo
		gazebo_packages="gz-garden"
	else
		# Expects Ubuntu 22.04 > by default
		echo "Gazebo (Harmonic) will be installed"
		echo "Earlier versions will be removed"
		# Add Gazebo binary repository
		sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
		echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
		sudo apt-get update -y --quiet

		# Install Gazebo
		gazebo_packages="gz-harmonic libunwind-dev"

		if [[ "${UBUNTU_RELEASE}" == "24.04" ]]; then
			gazebo_packages="$gazebo_packages cppzmq-dev"
		fi
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

	if sudo dmidecode -t system | grep -q "Manufacturer: VMware, Inc." ; then
		# fix VMWare 3D graphics acceleration for gazebo
		echo "export SVGA_VGPU10=0" >> ~/.profile
	fi

fi
