#! /usr/bin/env bash

INSTALL_NUTTX="true"
INSTALL_SIM="true"

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
	apt-get --quiet -y update && apt-get --quiet -y install \
		ca-certificates \
		curl \
		gnupg \
		gosu \
		lsb-core \
		sudo \
		wget \
		;
fi

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# check ubuntu version
# instructions for 16.04, 18.04
# otherwise warn and point to docker?
UBUNTU_RELEASE=`lsb_release -rs`

if [[ "${UBUNTU_RELEASE}" == "14.04" ]]; then
	echo "Ubuntu 14.04 unsupported, see docker px4io/px4-dev-base"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
	echo "Ubuntu 16.04"
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
	echo "Ubuntu 18.04"
fi

export DEBIAN_FRONTEND=noninteractive

echo "Installing PX4 general dependencies"

sudo apt-get update -yy --quiet
sudo apt-get -yy --quiet --no-install-recommends install \
	astyle \
	build-essential \
	ccache \
	clang \
	clang-tidy \
	cmake \
	cppcheck \
	doxygen \
	file \
	g++ \
	gcc \
	gdb \
	git \
	lcov \
	make \
	ninja-build \
	python3-pip \
	python3-pygments \
	python3-setuptools \
	rsync \
	shellcheck \
	unzip \
	wget \
	xsltproc \
	zip \
	;


# python dependencies
echo "Installing PX4 Python dependencies"

# TODO: deprecate python 2
sudo python -m pip install --upgrade pip setuptools wheel
sudo python -m pip install -r ${DIR}/requirements.txt

sudo python3 -m pip install --upgrade pip setuptools wheel
sudo python3 -m pip install -r ${DIR}/requirements.txt


# NuttX toolchain (arm-none-eabi-gcc)
if [[ $INSTALL_NUTTX == "true" ]]; then

	echo "Installing NuttX dependencies"

	sudo apt-get -yy --quiet --no-install-recommends install \
		autoconf \
		automake \
		bison \
		bzip2 \
		flex \
		gdb-multiarch \
		gperf \
		libncurses-dev \
		libtool \
		pkg-config \
		vim-common \
		;

	# add user to dialout group (serial port access)
	sudo usermod -a -G dialout $USER


	# arm-none-eabi-gcc
	NUTTX_GCC_VERSION="7-2017-q4-major"

	GCC_VER_STR=$(arm-none-eabi-gcc --version)
	STATUSRETVAL=$(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}")

	if [ $STATUSRETVAL -eq "1" ]; then
		echo "arm-none-eabi-gcc-${NUTTX_GCC_VERSION} found, skipping installation"

	else
		echo "Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		wget -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 && \
			sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/;

		# add arm-none-eabi-gcc to user's PATH
		exportline="export PATH=/opt/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}/bin:\$PATH"

		if grep -Fxq "$exportline" $HOME/.profile;
		then
			echo "${NUTTX_GCC_VERSION} path already set.";
		else
			echo $exportline >> $HOME/.profile;
		fi
	fi

fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then

	echo "Installing PX4 simulation dependencies"

	# java (jmavsim or fastrtps)
	sudo apt-get -yy --quiet --no-install-recommends install \
		ant \
		default-jre-headless \
		default-jdk-headless \
		;

	# Gazebo
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	sudo apt-get update -yy --quiet
	sudo apt-get -yy --quiet --no-install-recommends install gazebo9

fi
