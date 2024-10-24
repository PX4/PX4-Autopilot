#! /usr/bin/env bash

set -e

## Bash script to setup PX4 development environment on Ubuntu LTS (24.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for NuttX, and Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## - Gazebo simulator (omit with arg: --no-sim-tools)
## Options:
## - To run within docker with --from-docker arg

INSTALL_NUTTX="true"
INSTALL_SIM="true"
INSTALL_ARCH=`uname -m`
INSIDE_DOCKER="false"

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--no-nuttx" ]]; then
		INSTALL_NUTTX="false"
	fi

	if [[ $arg == "--no-sim-tools" ]]; then
		INSTALL_SIM="false"
	fi

	if [[ $arg == "--from-docker" ]]; then
		# Future proofing the script, please dont remove
		INSIDE_DOCKER="true"
	fi

	if [[ $arg == "--help" ]]; then
		echo "#⚡️ PX4 Dependency Installer for Ubuntu"
		echo "# Options:
		#
		#  --no-nuttx       boolean
		#  --no-sim-tools   boolean"
		echo "#"
		exit
	fi
done

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# check requirements.txt exists (script not run in source tree)
REQUIREMENTS_FILE="requirements.txt"
if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
	echo "FAILED: ${REQUIREMENTS_FILE} needed in same directory as ubuntu.sh (${DIR})."
	return 1
fi

# check ubuntu version
UBUNTU_RELEASE="`lsb_release -rs`"

VERBOSE_BAR="####################"
echo
echo $VERBOSE_BAR
echo "#⚡️ Starting PX4 Dependency Installer for Ubuntu ${UBUNTU_RELEASE} (${INSTALL_ARCH})"
echo "# Options:
#
#  - Install NuttX = ${INSTALL_NUTTX}
#  - Install Simulation = ${INSTALL_SIM}"
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
echo $VERBOSE_BAR
echo "🍻 Installing PX4 Python3 dependencies"
echo $VERBOSE_BAR
echo

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
		kconfig-frontends \
		libelf-dev \
		libexpat-dev \
		libgmp-dev \
		libisl-dev \
		libmpc-dev \
		libmpfr-dev \
		libncurses6 \
		libncurses-dev \
		libncursesw6 \
		libtool \
		pkg-config \
		screen \
		texinfo \
		u-boot-tools \
		util-linux \
		vim-common \
		gcc-arm-linux-gnueabihf \
		gcc-aarch64-linux-gnu \
		;

	if [ -n "$USER" ]; then
		# add user to dialout group (serial port access)
		sudo usermod -aG dialout $USER
	fi

	# arm-none-eabi-gcc
	NUTTX_GCC_VERSION="9-2020-q2-update"
	NUTTX_GCC_VERSION_SHORT="9-2020q2"
	echo
	echo $VERBOSE_BAR
	echo "🍻 Verifying proper gcc version (${NUTTX_GCC_VERSION}), and installing if not found"
	echo

	source $HOME/.profile
	if [ $(which arm-none-eabi-gcc) ]; then
		GCC_VER_STR=$(arm-none-eabi-gcc --version)
		GCC_FOUND_VER=$(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}" || true)
	fi

	if [[ "$GCC_FOUND_VER" == "1" ]]; then
		# echo "arm-none-eabi-gcc-${NUTTX_GCC_VERSION} found, skipping installation"
		echo "📌 Skipping installation, the arm cross compiler was found"
		echo $VERBOSE_BAR
		echo

	else
		# echo "Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		echo "📌 The arm cross compiler was not found";
		echo " * Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		COMPILER_NAME="gcc-arm-none-eabi-${NUTTX_GCC_VERSION}"
		COMPILER_PATH="/tmp/$COMPILER_NAME-linux.tar.bz2"
		wget -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/${NUTTX_GCC_VERSION_SHORT}/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-${INSTALL_ARCH}-linux.tar.bz2

		if [[ $INSIDE_DOCKER ]]; then
			sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/;
			# add arm-none-eabi-gcc to user's PATH
			exportline="export PATH=/opt/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}/bin:\$PATH"

			if grep -Fxq "$exportline" $HOME/.profile; then
				echo "${NUTTX_GCC_VERSION} path already set.";
			else
				echo $exportline >> $HOME/.profile;
				source $HOME/.profile; # Allows to directly build NuttX targets in the same terminal
			fi
			echo " * arm-none-eabi-gcc (${NUTTX_GCC_VERSION}) Installed Succesful to /opt/${COMPILER_NAME}/bin"
			echo $VERBOSE_BAR
			echo
		else
			sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /tmp/;
			sudo cp /opt/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}/bin/* /usr/bin
			echo " * arm-none-eabi-gcc (${NUTTX_GCC_VERSION}) Installed Succesful to /usr/bin"
			echo $VERBOSE_BAR
			echo
		fi
	fi
fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then

	echo
	echo $VERBOSE_BAR
	echo "🍻 Installing PX4 Simulation Tools"
	echo

	# General simulation dependencies
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		bc \
		ant \
		libvecmath-java \
		;

	echo "Gazebo (Harmonic) will be installed"
	echo "Earlier versions will be removed"
	# Add Gazebo binary repository
	sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	sudo apt-get update -y --quiet

	# Install Gazebo
	gazebo_packages="gz-harmonic"

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

if [[ $INSIDE_DOCKER == "false" ]] && [[ $INSTALL_NUTTX == "true" ]]; then
	echo
	echo $VERBOSE_BAR
	echo "💡 We recommend you relogin/reboot before attempting to build NuttX targets"
	echo $VERBOSE_BAR
	echo
fi

echo
echo
echo $VERBOSE_BAR
echo "#⚡️ PX4 Dependency Installer Ended Succesfully
#
#  For more information on PX4 Autopilot check out our docs
#  at docs.px4.io, if you find a bug please file an issue
#  on GitHub https://github.com/px4/px4-autopilot"
echo $VERBOSE_BAR
echo
