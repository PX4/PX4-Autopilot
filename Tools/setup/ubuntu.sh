#! /usr/bin/env bash

set -e

## Bash script to setup PX4 development environment on Ubuntu LTS (20.04, 18.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for NuttX, jMAVSim, Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## - jMAVSim and Gazebo simulator (omit with arg: --no-sim-tools)
## Optional:
## - FastRTPS and FastCDR (with args: --with-rtps)

INSTALL_NUTTX="true"
INSTALL_SIM="true"
INSTALL_ARCH=`uname -m`
INSTALL_RTPS="false"
INSTALL_JAVA="false"
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

	if [[ $arg == "--with-rtps" ]]; then
		INSTALL_RTPS="true"
	fi

	if [[ $arg == "--with-java" ]]; then
		INSTALL_JAVA="true"
	fi

	if [[ $arg == "--from-docker" ]]; then
		INSIDE_DOCKER="true"
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
# otherwise warn and point to docker?
UBUNTU_RELEASE="`lsb_release -rs`"

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

VERBOSE_BAR="####################"
echo
echo $VERBOSE_BAR
echo "#⚡️ Starting PX4 Dependency Installer for Ubuntu ${UBUNTU_RELEASE} (${INSTALL_ARCH})"
echo "# Options:
#
#  - Install NuttX = ${INSTALL_NUTTX}
#  - Install Java = ${INSTALL_JAVA}
#  - Install Simulation = ${INSTALL_SIM}
#  - Install RTPS = ${INSTALL_RTPS}"
echo $VERBOSE_BAR
echo

echo
echo $VERBOSE_BAR
echo "🍻 Installing System Dependencies"
echo $VERBOSE_BAR
echo

sudo apt-get update -y --quiet
sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
	build-essential \
  g++ \
  gcc \
  gdb \
  astyle \
  cmake \
  cppcheck \
  file \
  git \
  lcov \
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
  libssl-dev \
  ;

# Python 3 dependencies
echo
echo $VERBOSE_BAR
echo "🍻 Installing Python dependencies"
echo $VERBOSE_BAR
echo

if [ -n "$VIRTUAL_ENV" ]; then
	# virtual envrionments don't allow --user option
  python -m pip install -r ${DIR}/requirements.txt
else
	# older versions of Ubuntu require --user option
  if [[ $INSIDE_DOCKER == "true" ]]; then
    # when running inside a docker container we don't need to install
    # under --user since the installer user is root
    # its best to install packages globaly for any user to find
    python3 -m pip install -r /tmp/requirements.txt
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
    vim-common \
    g++-arm-linux-gnueabihf \
    gcc-arm-linux-gnueabihf \
    g++-aarch64-linux-gnu \
    gcc-aarch64-linux-gnu \
    ;

  if [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
    sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    kconfig-frontends \
    ;
  fi


	if [ -n "$USER" ]; then
		# add user to dialout group (serial port access)
		sudo usermod -a -G dialout $USER
	fi

  NUTTX_GCC_VERSION="9-2020-q2-update"
  NUTTX_GCC_VERSION_SHORT="9-2020q2"
  echo
  echo $VERBOSE_BAR
  echo "🍻 Verifying proper gcc version (${NUTTX_GCC_VERSION}), and installing if not found"
  echo

  source $HOME/.profile # load changed path for the case the script is reran before relogin
	if [ $(which arm-none-eabi-gcc) ]; then
    GCC_VER_STR=$(arm-none-eabi-gcc --version)
    GCC_VER_FOUND=$(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}")
  fi

  if [[ $(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}") == "1" ]]; then
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
      wget -O $COMPILER_PATH https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/${NUTTX_GCC_VERSION_SHORT}/${COMPILER_NAME}-${INSTALL_ARCH}-linux.tar.bz2
    fi
    sudo tar -jxf $COMPILER_PATH -C /opt/;

    # add arm-none-eabi-gcc to user's PATH
    exportline="export PATH=\"/opt/${COMPILER_NAME}/bin:\$PATH\""
    if [[ $INSIDE_DOCKER == "true" ]]; then
      # when running on a docker container its best to set the environment globally
      # since we don't know which user is going to be running commands on the container
      touch /etc/profile.d/px4env.sh
      echo $exportline >> /etc/profile.d/px4env.sh
    else
      if grep -Fxq "$exportline" $HOME/.profile; then
        echo "${NUTTX_GCC_VERSION} path already set.";
      else
        echo $exportline >> $HOME/.profile;
      fi
    fi
    echo " * arm-none-eabi-gcc (${NUTTX_GCC_VERSION}) Installed Succesful to /opt/${COMPILER_NAME}/bin"
    echo $VERBOSE_BAR
	  echo
  fi
fi

# Install JAVA
if [[ $INSTALL_JAVA == "true" ]]; then
  JDK_VERSION="14.0.2_12"
  echo
  echo $VERBOSE_BAR
  echo "🍻 Installing Java JDK

  * Version: $JDK_VERSION
  * Path: /opt/jdk-14.0.2+12"
  echo $VERBOSE_BAR
  echo

  JDK_DOWNLOAD="/tmp/OpenJDK14U-jdk_x64_linux_hotspot_$JDK_VERSION.tar.gz"
  wget -O $JDK_DOWNLOAD https://github.com/AdoptOpenJDK/openjdk14-binaries/releases/download/jdk-14.0.2%2B12/OpenJDK14U-jdk_x64_linux_hotspot_14.0.2_12.tar.gz
  sudo tar -xzf $JDK_DOWNLOAD -C /opt/
  export PATH="/opt/jdk-14.0.2+12/bin:$PATH"
fi

# Fast-RTPS
if [[ $INSTALL_RTPS == "true" ]]; then
  echo
  echo $VERBOSE_BAR
  echo "🍻 Installing Fast-RTPS"
  echo $VERBOSE_BAR
  echo

  GRADLE_VERSION="6.4.1"
  wget -O "/tmp/gradle-$GRADLE_VERSION-bin.zip" "https://services.gradle.org/distributions/gradle-$GRADLE_VERSION-bin.zip" \
    && unzip -d /opt/gradle "/tmp/gradle-$GRADLE_VERSION-bin.zip"
  export PATH="$PATH:/opt/gradle/gradle-$GRADLE_VERSION/bin"

  # Intall foonathan_memory from source as it is required to Fast-RTPS >= 1.9
  git clone https://github.com/eProsima/foonathan_memory_vendor.git /tmp/foonathan_memory \
    && cd /tmp/foonathan_memory \
    && mkdir build && cd build \
    && cmake .. \
    && cmake --build . --target install -- -j $(nproc)

  # Fast-DDS (Fast-RTPS 2.1.1)
  git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.1.1 /tmp/FastRTPS-2.1.1 \
    && cd /tmp/FastRTPS-2.1.1 \
    && mkdir build && cd build \
    && cmake -DTHIRDPARTY=ON -DSECURITY=ON .. \
    && cmake --build . --target install -- -j $(nproc)

  # Fast-RTPS-Gen 1.0.4
  git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 /tmp/Fast-RTPS-Gen-1.0.4 \
    && cd /tmp/Fast-RTPS-Gen-1.0.4 \
    && gradle assemble \
    && gradle install

fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then

  echo
  echo $VERBOSE_BAR
  echo "🍻 Installing PX4 Simulation Tools"
  echo

  if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
    gazebo_version=9
  elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
    gazebo_version=11
  fi

  echo "  * Gazebo Version $gazebo_version"
  echo $VERBOSE_BAR

  # General simulation dependencies
  sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    bc \
    ant \
    libvecmath-java \
    ;

  # Installing Gazebo and dependencies
  # Setup OSRF Gazebo repository
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  # Update list, since new gazebo-stable.list has been added
  sudo apt-get update -y --quiet
  sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
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

if [[ $INSIDE_DOCKER == "true" ]]; then
  # cleanup installation
  rm -rf /tmp/
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
