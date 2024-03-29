#!/bin/bash

## Bash script for setting up a PX4 development environment for Pixhawk/NuttX targets in Bash on Windows.
## It can be used for installing the NuttX toolchain (only).
##
## Installs:
## - Common dependencies and tools for all targets (including: Ninja build system, pyulog)
## - FastRTPS and FastCDR
## - NuttX toolchain (i.e. 64 bit version of gcc compiler)
## - PX4/Firmware source (to ~/src/Firmware/)

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &


# Ninja build system
ninja_dir=$HOME/ninja
echo "Installing Ninja to: $ninja_dir."
if [ -d "$ninja_dir" ]
then
    echo " Ninja already installed."
else
    pushd .
    mkdir -p $ninja_dir
    cd $ninja_dir
    wget https://github.com/martine/ninja/releases/download/v1.6.0/ninja-linux.zip
    unzip ninja-linux.zip
    rm ninja-linux.zip
    exportline="export PATH=$ninja_dir:\$PATH"
    if grep -Fxq "$exportline" ~/.profile; then echo " Ninja already in path" ; else echo $exportline >> ~/.profile; fi
    . ~/.profile
    popd
fi


# Common Dependencies
echo "Installing common dependencies"
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git git-core wget zip python-empy python-toml python-numpy cmake build-essential genromfs vim-common -y
# required python packages
sudo apt-get install python-dev -y
sudo apt-get install python-pip -y
sudo -H pip install pandas jinja2
pip install pyserial
# optional python tools
pip install pyulog
echo "Installing jMAVSim simulator dependencies"
sudo apt-get install ant -y

# NuttX
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y

# Install Java (needed by fastrtpsgen)
## Java7
sudo apt-get install default-jdk -y
# Install FastRTPS 1.5.0 and FastCDR-1.0.7
fastrtps_dir=$HOME/eProsima_FastRTPS-1.5.0-Linux
echo "Installing FastRTPS to: $fastrtps_dir"
if [ -d "$fastrtps_dir" ]
then
    echo " FastRTPS already installed."
else
    pushd .
    cd ~
    wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz
    mv eprosima_fastrtps-1-5-0-linux-tar-gz eprosima_fastrtps-1-5-0-linux.tar.gz
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
    tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
    (cd eProsima_FastCDR-1.0.7-Linux && ./configure --libdir=/usr/lib && make && sudo make install)
    (cd eProsima_FastRTPS-1.5.0-Linux && ./configure --libdir=/usr/lib && make && sudo make install)
    exportline="export FASTRTPSGEN_DIR=/usr/local/bin/"
    if grep -Fxq "$exportline" ~/.bashrc; then echo " fastrtpsgen path already set." ; else echo $exportline >> ~/.bashrc; fi
    . ~/.bashrc
    popd
fi


# Clean up old GCC
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded -y
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa -y

# GNU Arm Embedded Toolchain: 7-2017-q4-major December 18, 2017
gcc_dir=$HOME/gcc-arm-none-eabi-7-2017-q4-major
echo "Installing GCC to: $gcc_dir"
if [ -d "$gcc_dir" ]
then
    echo " GCC already installed."
else
    pushd .
    cd ~    
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
    tar -jxf gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
    exportline="export PATH=$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:\$PATH"
    if grep -Fxq "$exportline" ~/.profile; then echo " GCC path already set." ; else echo $exportline >> ~/.profile; fi
    . ~/.profile
    popd
fi


# Clone PX4/Firmware
clone_dir=~/src
echo "Cloning PX4 to: $clone_dir."
if [ -d "$clone_dir" ]
then
    echo " Firmware already cloned."
else
    mkdir -p $clone_dir
    cd $clone_dir
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
fi
cd $clone_dir/Firmware


