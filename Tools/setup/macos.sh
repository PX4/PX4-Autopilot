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
## installs from bottles even while OSRF has them pulled, and protobuf to
## protobuf-pin.txt so those bottles' headers still compile.
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
	# Simulation packages. This replaces the px4-sim / px4-sim-gazebo
	# meta-formulae, which declared cross-tap dependencies that
	# Homebrew 4.5+ no longer auto-resolves. Same migration pattern as
	# the toolchain block above. See PX4/homebrew-px4#104 for the
	# px4-dev precedent.
	#
	# osrf/simulation: gz-harmonic (Gazebo Harmonic meta-formula)
	#
	# Trust before tapping, same as the toolchain taps above. Tapping an
	# untrusted tap fails, which leaves no tap clone to pin below; the
	# later `brew install osrf/simulation/gz-harmonic` then taps it
	# implicitly at HEAD and the pin is silently skipped.
	if brew trust --help &> /dev/null; then
		brew trust osrf/simulation
	fi

	brew_tap osrf/simulation

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
		git -C "$GZ_TAP_DIR" fetch --quiet origin "$GZ_TAP_PIN" 2>/dev/null || true
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

	# Gazebo's generated headers only compile against the exact protobuf
	# their gencode came from, so protobuf has to come from the
	# homebrew-core revision that was current when the pinned gz bottles
	# were built rather than from whatever homebrew-core ships today. See
	# protobuf-pin.txt.
	#
	# This runs after the installs above rather than before them because
	# opencv@4 depends on protobuf as well and drags in the current one:
	# brew resolves a dependency against the versions recorded in the
	# dependent's bottle, so an opencv bottle rebuilt against a newer
	# protobuf pulls that protobuf in no matter what is installed.
	# Pinning the formula instead of reinstalling it here is not an
	# option either: brew refuses to install anything whose pinned
	# dependency is not the current one ("You must `brew unpin
	# protobuf`").
	PROTOBUF_FORMULA="Formula/p/protobuf.rb"
	PROTOBUF_PIN_LINE=$(grep -v -e '^#' -e '^[[:space:]]*$' "${DIR}/protobuf-pin.txt" | head -n 1)
	read -r PROTOBUF_PIN PROTOBUF_PIN_VERSION <<< "$PROTOBUF_PIN_LINE" || true
	if [[ -n $PROTOBUF_PIN ]]; then
		CORE_TAP_DIR=$(brew --repo homebrew/core)
		INSTALLED_PROTOBUF=$(brew list --versions protobuf 2> /dev/null | awk '{print $2}')
		if [[ $INSTALLED_PROTOBUF == "$PROTOBUF_PIN_VERSION" ]]; then
			echo "[macos.sh] protobuf ${PROTOBUF_PIN_VERSION} is what the gz bottles need, leaving it"
		elif ! git -C "$CORE_TAP_DIR" rev-parse --git-dir &> /dev/null; then
			# homebrew-core resolves through the JSON API by default, and a
			# formula as it was at some commit can only be read from a clone.
			echo "[macos.sh] WARNING: homebrew-core is not cloned here, so protobuf cannot be held" \
				"at ${PROTOBUF_PIN_VERSION} and Gazebo's headers may fail to compile." \
				"Run 'brew tap homebrew/core' once to enable the pin."
		else
			echo "[macos.sh] Installing protobuf ${PROTOBUF_PIN_VERSION} from homebrew-core ${PROTOBUF_PIN}"
			PROTOBUF_TMP=$(mktemp -d)
			# A clone that has not been updated since the pin was made does
			# not have the commit yet.
			git -C "$CORE_TAP_DIR" fetch --quiet origin "$PROTOBUF_PIN" 2> /dev/null || true
			if git -C "$CORE_TAP_DIR" show "${PROTOBUF_PIN}:${PROTOBUF_FORMULA}" \
				> "${PROTOBUF_TMP}/pinned.rb" 2> /dev/null; then
				# Swap the formula in place rather than checking it out, so
				# the clone's git state is never touched and the file comes
				# back byte for byte whether or not the install works.
				cp "${CORE_TAP_DIR}/${PROTOBUF_FORMULA}" "${PROTOBUF_TMP}/current.rb"
				cp "${PROTOBUF_TMP}/pinned.rb" "${CORE_TAP_DIR}/${PROTOBUF_FORMULA}"
				# Whatever is installed is the wrong version, and only
				# reinstall replaces it; install alone would be a no-op.
				if [[ -n $INSTALLED_PROTOBUF ]]; then
					PROTOBUF_INSTALL="reinstall"
				else
					PROTOBUF_INSTALL="install"
				fi
				# brew reads this clone instead of the JSON API only with
				# HOMEBREW_NO_INSTALL_FROM_API, and brew update would put
				# the swapped formula back before the install. Without
				# HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK brew would then
				# see everything linked against the protobuf it just
				# replaced as broken and reinstall it, which pulls the
				# newer protobuf straight back in through opencv@4.
				if ! HOMEBREW_NO_INSTALL_FROM_API=1 HOMEBREW_NO_AUTO_UPDATE=1 \
					HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK=1 \
					brew "$PROTOBUF_INSTALL" protobuf; then
					echo "[macos.sh] WARNING: could not install protobuf ${PROTOBUF_PIN_VERSION}," \
						"continuing on the current one (Gazebo headers may fail to compile)"
				fi
				cp "${PROTOBUF_TMP}/current.rb" "${CORE_TAP_DIR}/${PROTOBUF_FORMULA}"
			else
				echo "[macos.sh] WARNING: homebrew-core commit ${PROTOBUF_PIN} is not available," \
					"continuing on the current protobuf (Gazebo headers may fail to compile)"
			fi
			rm -rf "$PROTOBUF_TMP"
		fi
	fi

	# XQuartz is required for Gazebo GUI display on macOS.
	if ! brew list --cask xquartz &> /dev/null; then
		echo "[macos.sh] Installing XQuartz (required for Gazebo display)"
		# XQuartz is not in the pinned package repos.
		env -u HOMEBREW_NO_INSTALL_FROM_API brew install --cask xquartz
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
