#! /usr/bin/env bash

## Basch script to setup the PX4 development environment on macOS
## Works for Intel and Arm based Apple hardware
##
## Installs:
##	- Common dependencies and tools for building PX4
##	- Cross compilers for building hardware targets using NuttX
##	- Can also install the default simulation provided by the px4-sim homebrew
##		Formula
##
## For more information regarding the Homebrew Formulas see:
##		https://github.com/PX4/homebrew-px4/
##

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Reinstall if --reinstall set
REINSTALL_FORMULAS=""
# Install simulation tools?
INSTALL_SIM=""

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--reinstall" ]]; then
		REINSTALL_FORMULAS=$arg
	elif [[ $arg == "--sim-tools" ]]; then
		INSTALL_SIM=$arg
	fi
done

echo "[macos.sh] Installing the development dependencies for the PX4 Autopilot"

if ! command -v brew &> /dev/null
then
	# install Homebrew if not installed yet
	echo "[macos.sh] Installing Homebrew"
	/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
fi

# Install px4-dev formula
if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
	echo "[macos.sh] Re-installing dependencies (homebrew px4-dev)"

	# confirm Homebrew installed correctly
	brew doctor

	brew tap osx-cross/arm
	brew tap PX4/px4

	brew reinstall px4-dev
	brew link --overwrite --force arm-gcc-bin@13
else
	if brew ls --versions px4-dev > /dev/null; then
		echo "[macos.sh] px4-dev already installed"
	else
		echo "[macos.sh] Installing general dependencies (homebrew px4-dev)"

		brew tap osx-cross/arm
		brew tap PX4/px4

		brew install px4-dev
		brew link --overwrite --force arm-gcc-bin@13
	fi
fi

# Python dependencies
echo "[macos.sh] Installing Python3 dependencies"
# We need to have future to install pymavlink later.
python3 -m pip install future
python3 -m pip install --user -r ${DIR}/requirements.txt

# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
	if brew ls --versions px4-sim > /dev/null; then
		brew install px4-sim
	elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
		brew reinstall px4-sim
	fi
fi

echo "[macos.sh] All set! The PX4 Autopilot toolchain was installed."
