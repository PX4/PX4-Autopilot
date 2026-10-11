#! /usr/bin/env bash

## Basch script to setup the PX4 development environment on macOS
## Works for Intel and Arm based Apple hardware
##
## Installs:
##	- Common dependencies and tools for building PX4
##	- Cross compilers for building hardware targets using NuttX
##	- With --sim-tools: Gazebo Harmonic simulation stack
##
## --sim-tools installs Gazebo from the locked conda-forge environment in
## macos/pixi.toml, and loads it with the Python venv.
##
## Homebrew 4.5+ no longer auto-resolves cross-tap dependencies, so
## every tap and package is listed explicitly here rather than hidden
## behind meta-formulae. See PX4/homebrew-px4#104 for background.
##

# Abort on the first failing command.
set -e

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

# Leave a checkout that is already there. `brew tap` on one would try to
# unshallow it, and CI has already checked these repos out at a commit.
brew_tap() {
	local name="$1"
	local user="${name%%/*}"
	local repo="${name#*/}"
	local path
	path="$(brew --repo)/Library/Taps/${user}/homebrew-${repo}"
	if [[ -d "${path}/.git" ]]; then
		return 0
	fi
	brew tap "$name"
}

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
	brew uninstall flock 2>/dev/null || true
	brew untap discoteq/discoteq
fi

# Required taps. Homebrew 4.5+ no longer auto-resolves cross-tap
# dependencies, so every tap that a package lives in must be added
# explicitly here before `brew install`.
#
# - osx-cross/arm: arm-gcc-bin@13 (ARM cross-compiler)
# - PX4/px4:       fastdds, genromfs, kconfig-frontends (PX4-specific)
#
# Homebrew 6.0+ refuses to load formulae from third-party taps unless they
# are explicitly trusted ("Refusing to load formula ... from untrusted tap"),
# and recent versions validate every formula of a tap while tapping it. An
# untrusted tap therefore fails with "Cannot tap ...: invalid syntax in tap!",
# so the taps must be trusted *before* they are tapped. `brew trust` works on
# a tap that is not installed yet. Without the taps, `brew install` aborts
# on the first PX4/px4 formula before pouring any package (including ccache).
# `brew trust` only exists on Homebrew 6.0+; guard it so older versions,
# which don't gate untrusted taps, skip it silently.
if brew trust --help &> /dev/null; then
	brew trust osx-cross/arm
	brew trust PX4/px4
fi

brew_tap osx-cross/arm
brew_tap PX4/px4

# Package list. This replaces the px4-dev meta-formula, which is kept
# as a deprecated no-op upstream. See PX4/homebrew-px4 for history.
PX4_BREW_PACKAGES=(
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
	brew doctor || true # warnings are informational here
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
	# Gazebo and everything the gz modules link against (OpenCV for
	# PX4-OpticalFlow, GStreamer for the camera plugin) come from the
	# locked conda-forge environment in macos/pixi.toml instead of
	# Homebrew. conda-forge never removes or rebuilds a published
	# package, so the lock keeps installing the same gz, protobuf and
	# abseil binaries. See macos/pixi.toml.
	PX4_SIM_BREW_PACKAGES=(
		exiftool
		glog
		graphviz
		pixi
	)

	if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
		echo "[macos.sh] Re-installing PX4 simulation dependencies"
		brew reinstall "${PX4_SIM_BREW_PACKAGES[@]}"
	else
		echo "[macos.sh] Installing PX4 simulation dependencies"
		brew install "${PX4_SIM_BREW_PACKAGES[@]}"
	fi

	PIXI_MANIFEST="${DIR}/macos/pixi.toml"
	echo "[macos.sh] Installing Gazebo from ${PIXI_MANIFEST}"
	pixi install --locked --manifest-path "$PIXI_MANIFEST"

	# Load the Gazebo environment with the venv, so developers keep a
	# single activation step. This is written by hand rather than taken
	# from `pixi shell-hook`, which exports the PATH of the shell running
	# this script instead of prepending to the user's, and sources every
	# package's bash completions. The activate.d scripts set the paths
	# the gz CLI (ruby gems) and renderer (OGRE) need. CMake needs the
	# prefix spelled out: the gz config files are found through PATH, but
	# the find_path(zmq.hpp) in gz-cmake's FindCPPZMQ is not. The env
	# ships its own python, so the venv's bin goes back in front of it
	# afterwards.
	GZ_ENV_SCRIPT="$VENV_DIR/bin/px4-gz-env.sh"
	cat > "$GZ_ENV_SCRIPT" <<-EOF
		export CONDA_PREFIX="${DIR}/macos/.pixi/envs/default"
		export PATH="\$CONDA_PREFIX/bin:\$PATH"
		export CMAKE_PREFIX_PATH="\$CONDA_PREFIX\${CMAKE_PREFIX_PATH:+:\$CMAKE_PREFIX_PATH}"
		for f in "\$CONDA_PREFIX"/etc/conda/activate.d/*.sh; do
			. "\$f"
		done
		unset f
	EOF
	GZ_ENV_MARKER="# px4: load the Gazebo environment"
	if ! grep -qF "$GZ_ENV_MARKER" "$VENV_DIR/bin/activate"; then
		cat >> "$VENV_DIR/bin/activate" <<-EOF

		$GZ_ENV_MARKER
		. "\$VIRTUAL_ENV/bin/px4-gz-env.sh"
		PATH="\$VIRTUAL_ENV/bin:\$PATH"
		export PATH
		EOF
	fi

	# XQuartz is required for Gazebo GUI display on macOS.
	if ! brew list --cask xquartz &> /dev/null; then
		echo "[macos.sh] Installing XQuartz (required for Gazebo display)"
		# XQuartz is not in the pinned package repos.
		env -u HOMEBREW_NO_INSTALL_FROM_API brew install --cask xquartz
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
