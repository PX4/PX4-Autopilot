#! /usr/bin/env bash

## Bash script to setup PX4 development environment on Ubuntu LTS (20.04, 18.04, 16.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for nuttx, jMAVSim, Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## - jMAVSim and Gazebo9 simulator (omit with arg: --no-sim-tools)
##
## Not Installs:
## - FastRTPS and FastCDR

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
	apt-get -qq update && DEBIAN_FRONTEND=noninteractive apt-get -q -y --no-install-recommends install \
		ca-certificates \
		gosu \
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
UBUNTU_RELEASE=$(cat /etc/os-release | grep VERSION_ID | cut -d "\"" -f 2)

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

fi


echo
echo "Installing PX4 general dependencies"

sudo apt-get -qq update
sudo DEBIAN_FRONTEND=noninteractive apt-get -q -y --no-install-recommends install \
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
	make \
	ninja-build \
	python3 \
	python3-dev \
	python3-pip \
	python3-setuptools \
	python3-wheel \
	rsync \
	shellcheck \
	;

if [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
	echo "Installing Ubuntu 16.04 PX4-compatible ccache version"
	wget -q -O /tmp/ccache_3.4.1-1_amd64.deb http://launchpadlibrarian.net/356662933/ccache_3.4.1-1_amd64.deb
	sudo dpkg -i /tmp/ccache_3.4.1-1_amd64.deb
fi

# Python3 dependencies
echo
echo "Installing PX4 Python3 dependencies"
if [ -f /.dockerenv ]; then
	# system wide for docker
	python3 -m pip install -r ${DIR}/requirements.txt
else
	python3 -m pip install --user --quiet -r ${DIR}/requirements.txt
fi

# NuttX toolchain (arm-none-eabi-gcc)
if [[ $INSTALL_NUTTX == "true" ]]; then

	echo
	echo "Installing NuttX dependencies"

	sudo DEBIAN_FRONTEND=noninteractive apt-get -q -y --no-install-recommends install \
		g++-multilib \
		gcc-multilib \
		gdb-multiarch \
		genromfs \
		pkg-config \
		screen \
		vim-common \
		;

	if [ -n "$USER" ]; then
		# add user to dialout group (serial port access)
		sudo usermod -a -G dialout $USER
	fi

	# arm-none-eabi-gcc
	NUTTX_GCC_VERSION="9-2020-q2-update"
	NUTTX_GCC_VERSION_SHORT="9-2020q2"

	source $HOME/.profile # load changed path for the case the script is reran before relogin
	if [ $(which arm-none-eabi-gcc) ]; then
		GCC_VER_STR=$(arm-none-eabi-gcc --version)
		GCC_FOUND_VER=$(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}")
	fi

	if [[ "$GCC_FOUND_VER" == "1" ]]; then
		echo "arm-none-eabi-gcc-${NUTTX_GCC_VERSION} found, skipping installation"

	else
		echo "Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		wget -q -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/${NUTTX_GCC_VERSION_SHORT}/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-${INSTALL_ARCH}-linux.tar.bz2 && \
			sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/;

		# add arm-none-eabi-gcc to user's PATH
		exportline="export PATH=/opt/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}/bin:\$PATH"

		if grep -Fxq "$exportline" $HOME/.profile; then
			echo "${NUTTX_GCC_VERSION} path already set.";
		else
			echo $exportline >> $HOME/.profile;
		fi
	fi
fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then

	echo
	echo "Installing PX4 simulation dependencies"

	# General simulation dependencies
	sudo DEBIAN_FRONTEND=noninteractive apt-get -q -y --no-install-recommends install \
		bc \
		;

	if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
		gazebo_version=9
		MAVSDK_VERSION=0.39.0
		wget -q "https://github.com/mavlink/MAVSDK/releases/download/v${MAVSDK_VERSION}/mavsdk_{MAVSDK_VERSION})_ubuntu18.04_amd64.deb" -O /tmp/mavsdk.deb && sudo dpkg -i /tmp/mavsdk.deb
	elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
		gazebo_version=11
		MAVSDK_VERSION=0.39.0
		wget -q "https://github.com/mavlink/MAVSDK/releases/download/v{MAVSDK_VERSION}/mavsdk_{MAVSDK_VERSION}_ubuntu20.04_amd64.deb" -O /tmp/mavsdk.deb && sudo dpkg -i /tmp/mavsdk.deb
	else
		gazebo_version=11
	fi

	# Java (jmavsim or fastrtps)
	sudo DEBIAN_FRONTEND=noninteractive apt-get -q -y --no-install-recommends install \
		ant \
		default-jre-headless \
		default-jdk-headless \
		libvecmath-java \
		;

	# Gazebo
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget -q http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	# Update list, since new gazebo-stable.list has been added
	sudo apt-get update -qq
	sudo DEBIAN_FRONTEND=noninteractive apt-get -q -y --no-install-recommends install \
		dmidecode \
		gazebo$gazebo_version \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		libeigen3-dev \
		libgazebo$gazebo_version-dev \
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

if [[ $INSTALL_NUTTX == "true" ]]; then
	echo
	echo "Relogin or reboot computer before attempting to build NuttX targets"
fi
