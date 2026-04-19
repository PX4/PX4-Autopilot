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

# Required taps. Homebrew 4.5+ no longer auto-resolves cross-tap
# dependencies, so every tap that a package lives in must be added
# explicitly here before `brew install`.
#
# - osx-cross/arm: arm-gcc-bin@13 (ARM cross-compiler)
# - PX4/px4:       fastdds, genromfs, kconfig-frontends (PX4-specific)
brew tap osx-cross/arm
brew tap PX4/px4

# Package list. This replaces the px4-dev meta-formula, which is kept
# as a deprecated no-op upstream. See PX4/homebrew-px4 for history.
PX4_BREW_PACKAGES=(
	ant
	astyle
	bash-completion
	ccache
	cmake
	fastdds
	genromfs
	kconfig-frontends
	ncurses
	ninja
	osx-cross/arm/arm-gcc-bin@13
	python
	python-tk
)

if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
	echo "[macos.sh] Re-installing PX4 toolchain dependencies"
	brew doctor
	brew reinstall "${PX4_BREW_PACKAGES[@]}"
else
	echo "[macos.sh] Installing PX4 toolchain dependencies"
	brew install "${PX4_BREW_PACKAGES[@]}"
fi

brew link --overwrite --force arm-gcc-bin@13

# Python dependencies
echo "[macos.sh] Installing Python3 dependencies"
# We need to have future to install pymavlink later.
python3 -m pip install future
python3 -m pip install --user -r ${DIR}/requirements.txt

# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
	if ! brew ls --versions px4-sim > /dev/null; then
		brew install px4-sim
	elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
		brew reinstall px4-sim
	fi

	# jMAVSim requires a JDK (Java 17 LTS recommended)
	if ! brew ls --versions openjdk@17 > /dev/null; then
		echo "[macos.sh] Installing OpenJDK 17 (required for jMAVSim)"
		brew install openjdk@17
		sudo ln -sfn $(brew --prefix openjdk@17)/libexec/openjdk.jdk /Library/Java/JavaVirtualMachines/openjdk-17.jdk
	fi
fi

echo "[macos.sh] All set! The PX4 Autopilot toolchain was installed."
