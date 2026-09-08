#! /usr/bin/env bash

## Basch script to setup the PX4 development environment on macOS
## Works for Intel and Arm based Apple hardware
##
## Installs:
##	- Common dependencies and tools for building PX4
##	- Cross compilers for building hardware targets using NuttX
##	- With --sim-tools: Gazebo Harmonic and jMAVSim simulation stack
##
## --sim-tools pins the osrf/simulation tap to gz-tap-pin.txt so Gazebo
## installs from bottles even while OSRF has them pulled.
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

# discoteq/discoteq used to be the only source of flock (required by the
# NuttX apps archive step), but homebrew/core now carries the identical
# formula (same upstream, same version). Drop the old tap so `flock`
# resolves unambiguously from homebrew/core instead of erroring with
# "installed from the discoteq/discoteq tap but you are trying to install
# it from homebrew/core" on machines that still have it tapped.
if brew tap | grep -q '^discoteq/discoteq$'; then
	brew uninstall flock 2>/dev/null
	brew untap discoteq/discoteq
fi

# Required taps. Homebrew 4.5+ no longer auto-resolves cross-tap
# dependencies, so every tap that a package lives in must be added
# explicitly here before `brew install`.
#
# - osx-cross/arm: arm-gcc-bin@13 (ARM cross-compiler)
# - PX4/px4:       fastdds, genromfs, kconfig-frontends (PX4-specific)
brew tap osx-cross/arm
brew tap PX4/px4

# Homebrew 6.0+ refuses to load formulae from third-party taps unless they
# are explicitly trusted ("Refusing to load formula ... from untrusted tap").
# Trust each tap non-interactively before installing from it. Without this,
# `brew install` aborts before pouring any package (including ccache).
# `brew trust` only exists on Homebrew 6.0+; guard it so older versions,
# which don't gate untrusted taps, skip it silently.
if brew trust --help &> /dev/null; then
	brew trust osx-cross/arm
	brew trust PX4/px4
fi

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
	flock
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

	# OSRF drops the gz bottle blocks within minutes of a breaking
	# homebrew-core dependency bump and rebuilds them days later, so an
	# unpinned tap compiles Gazebo from source for a large part of the
	# year. Pin unconditionally so dev machines get the same fast, binary
	# install as CI. See gz-tap-pin.txt.
	GZ_TAP_PIN=$(grep -v '^#' "${DIR}/gz-tap-pin.txt" | tr -d '[:space:]')
	if [[ -n $GZ_TAP_PIN ]]; then
		GZ_TAP_DIR=$(brew --repo osrf/simulation)
		echo "[macos.sh] Pinning osrf/simulation to ${GZ_TAP_PIN}"
		# brew taps are shallow clones, so the pinned commit has to be
		# fetched by SHA before it can be checked out.
		git -C "$GZ_TAP_DIR" fetch --quiet origin "$GZ_TAP_PIN" 2>/dev/null
		if git -C "$GZ_TAP_DIR" checkout --quiet "$GZ_TAP_PIN"; then
			# `brew update` walks local taps and would reset the pin.
			# homebrew-core resolves through the JSON API, not this
			# clone, so nothing else goes stale.
			export HOMEBREW_NO_AUTO_UPDATE=1
		else
			echo "[macos.sh] WARNING: could not pin osrf/simulation to ${GZ_TAP_PIN}," \
				"continuing on tap HEAD (gz may build from source)"
		fi
	fi

	# Homebrew 6.0+ refuses to load formulae from untrusted third-party
	# taps (see the toolchain trust block above). Without this, the
	# gz-harmonic install aborts and the script still exits successfully,
	# leaving the simulation stack silently missing.
	if brew trust --help &> /dev/null; then
		brew trust osrf/simulation
	fi

	# opencv@4: the unversioned formula is OpenCV 5, which PX4-OpticalFlow
	# does not build against.
	PX4_SIM_BREW_PACKAGES=(
		exiftool
		glog
		graphviz
		gstreamer
		opencv@4
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
