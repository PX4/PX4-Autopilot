#! /usr/bin/env bash

## Basch script to setup the PX4 development environment on macOS
## Works for Intel and Arm based Apple hardware
##
## Installs:
##	- Common dependencies and tools for building PX4
##	- Cross compilers for building hardware targets using NuttX
##	- With --sim-tools: Gazebo Harmonic and jMAVSim simulation stack
##
## Homebrew 4.5+ no longer auto-resolves cross-tap dependencies, so
## every tap and package is listed explicitly here rather than hidden
## behind meta-formulae. See PX4/homebrew-px4#104 for background.
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

# Resolve to git repo root based on script location (handles submodules and subdirectory invocation)
ROOT_DIR="$(git -C "$DIR" rev-parse --show-toplevel 2>/dev/null || echo "$DIR")"
VENV_DIR="$ROOT_DIR/.venv"

# Create virtual environment if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
	echo "[macos.sh] Creating Python virtual environment at $VENV_DIR"
	python3 -m venv "$VENV_DIR"
fi

# We need to have future to install pymavlink later.
"$VENV_DIR/bin/pip" install future
"$VENV_DIR/bin/pip" install -r "${DIR}/requirements.txt"

# Register pre-commit git hooks
echo "[macos.sh] Installing git hooks (pre-commit)"
if [ -d "${ROOT_DIR}/.git" ] && [ -x "${VENV_DIR}/bin/pre-commit" ]; then
	(cd "${ROOT_DIR}" && "${VENV_DIR}/bin/pre-commit" install) || echo "[macos.sh] Note: 'pre-commit install' failed — run it manually from the repo root after activating the venv"
fi

# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
	# Simulation packages. This replaces the px4-sim / px4-sim-gazebo
	# meta-formulae, which declared cross-tap dependencies that
	# Homebrew 4.5+ no longer auto-resolves. Same migration pattern as
	# the toolchain block above. See PX4/homebrew-px4#104 for the
	# px4-dev precedent.
	#
	# osrf/simulation: gz-harmonic (Gazebo Harmonic meta-formula)
	brew tap osrf/simulation

	PX4_SIM_BREW_PACKAGES=(
		exiftool
		glog
		graphviz
		gstreamer
		opencv
		osrf/simulation/gz-harmonic
		protobuf
	)

	if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
		echo "[macos.sh] Re-installing PX4 simulation dependencies"
		brew reinstall "${PX4_SIM_BREW_PACKAGES[@]}"
	else
		echo "[macos.sh] Installing PX4 simulation dependencies"
		brew install "${PX4_SIM_BREW_PACKAGES[@]}"
	fi

	# XQuartz is required for Gazebo GUI display on macOS.
	if ! brew list --cask xquartz &> /dev/null; then
		echo "[macos.sh] Installing XQuartz (required for Gazebo display)"
		brew install --cask xquartz
	fi

	# jMAVSim requires a JDK (Java 17 LTS recommended)
	if ! brew ls --versions openjdk@17 > /dev/null; then
		echo "[macos.sh] Installing OpenJDK 17 (required for jMAVSim)"
		brew install openjdk@17
		sudo ln -sfn $(brew --prefix openjdk@17)/libexec/openjdk.jdk /Library/Java/JavaVirtualMachines/openjdk-17.jdk
	fi
fi

echo ""
echo "[macos.sh] All set! The PX4 Autopilot toolchain was installed."
echo ""
echo "Python dependencies were installed into a virtual environment at:"
echo "    $VENV_DIR"
echo ""
echo "Activate it before building (run in each new terminal session):"
echo "    source $VENV_DIR/bin/activate"
echo ""
