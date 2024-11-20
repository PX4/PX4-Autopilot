#! /usr/bin/env bash

#set -e

## Bash script to setup PX4 development environment on Ubuntu
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for PX4 development
## - NuttX toolchain (omit with arg: --no-nuttx)
## - Gazebo simulator (omit with arg: --no-sim-tools)
##

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


echo "Installing PX4 general dependencies"

sudo apt-get update -y --quiet
sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
	astyle \
	bc \
	build-essential \
	cmake \
	cppcheck \
	curl \
	file \
	g++ \
	gcc \
	gdb \
	git \
	gnupg \
	lcov \
	libssl-dev \
	libxml2-dev \
	libxml2-utils \
	lsb-release \
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

# PX4 python dependencies
echo "Installing PX4 dependencies"
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

# NuttX toolchain
if [[ $INSTALL_NUTTX == "true" ]]; then

	echo "Installing PX4 NuttX dependencies"

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

	sudo ${DIR}/ubuntu_gazebo.sh

fi
