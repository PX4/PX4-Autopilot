#!/bin/bash
################################################################################
# Copyright (c) 2026 ModalAI, Inc. All rights reserved.
#
# Builds a Debian package for the voxl2 target from existing build artifacts.
# Both modalai_voxl2 and modalai_voxl2-slpi must be built before running this.
#
# Usage:
#   ./Tools/packaging/voxl2/make_package.sh
#
################################################################################

set -e

################################################################################
# Resolve paths - script can be run from anywhere
################################################################################
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PKG_DIR="$SCRIPT_DIR/pkg"

BOARD_DIR="$PX4_DIR/boards/modalai/voxl2"
BUILD_APPS="$PX4_DIR/build/modalai_voxl2_default"
BUILD_SLPI="$PX4_DIR/build/modalai_voxl2-slpi_default"

################################################################################
# Check arguments
################################################################################
for arg in "$@"; do
	case "$(echo "$arg" | tr '[:upper:]' '[:lower:]')" in
		"-h"|"--help")
			echo ""
			echo " Build a Debian package for the voxl2 target."
			echo " You must build modalai_voxl2 and modalai_voxl2-slpi first."
			echo ""
			echo " Usage:"
			echo "   ./Tools/packaging/voxl2/make_package.sh"
			echo ""
			exit 0
			;;
		*)
			echo "Unknown argument: $arg"
			exit 1
			;;
	esac
done

################################################################################
# Derive version from git
################################################################################
cd "$PX4_DIR"
GIT_TAG=$(git describe --tags --abbrev=0 2>/dev/null || echo "0.0.0")
VERSION=$(echo "$GIT_TAG" | sed 's/^v//')
SHORT_HASH=$(git rev-parse --short HEAD)
VERSION="${VERSION}-g${SHORT_HASH}"

PACKAGE=$(grep "^Package:" "$PKG_DIR/control/control" | cut -d' ' -f 2)

echo "Package: $PACKAGE"
echo "Version: $VERSION"

################################################################################
# Validate build artifacts
################################################################################
MISSING=false

if [ ! -f "$BUILD_APPS/bin/px4" ]; then
	echo "Error: $BUILD_APPS/bin/px4 not found. Run 'make modalai_voxl2' first."
	MISSING=true
fi

if [ ! -f "$BUILD_APPS/bin/px4-alias.sh" ]; then
	echo "Error: $BUILD_APPS/bin/px4-alias.sh not found."
	MISSING=true
fi

if [ ! -f "$BUILD_SLPI/platforms/qurt/libpx4.so" ]; then
	echo "Error: $BUILD_SLPI/platforms/qurt/libpx4.so not found. Run 'make modalai_voxl2-slpi' first."
	MISSING=true
fi

if $MISSING; then
	exit 1
fi

################################################################################
# Clean old packaging artifacts
################################################################################
DATA_DIR="$PKG_DIR/data"
DEB_DIR="$PKG_DIR/DEB"

rm -rf "$DATA_DIR"
rm -rf "$DEB_DIR"
rm -f "$SCRIPT_DIR"/*.deb

mkdir -p "$DATA_DIR"

################################################################################
# Stage files into data directory
################################################################################

# --- Apps processor binary ---
mkdir -p "$DATA_DIR/usr/bin"
cp "$BUILD_APPS/bin/px4"          "$DATA_DIR/usr/bin/"
cp "$BUILD_APPS/bin/px4-alias.sh" "$DATA_DIR/usr/bin/"
chmod a+x "$DATA_DIR/usr/bin/px4"
chmod a+x "$DATA_DIR/usr/bin/px4-alias.sh"

# --- SLPI DSP library ---
mkdir -p "$DATA_DIR/usr/lib/rfsa/adsp"
cp "$BUILD_SLPI/platforms/qurt/libpx4.so" "$DATA_DIR/usr/lib/rfsa/adsp/"

# --- Target scripts ---
for script in voxl-px4 voxl-px4-start voxl-px4-hitl voxl-px4-hitl-start; do
	if [ -f "$BOARD_DIR/target/$script" ]; then
		cp "$BOARD_DIR/target/$script" "$DATA_DIR/usr/bin/"
		chmod a+x "$DATA_DIR/usr/bin/$script"
	fi
done

# --- Barometer calibration script ---
if [ -f "$BOARD_DIR/scripts/baro_temp_cal" ]; then
	cp "$BOARD_DIR/scripts/baro_temp_cal" "$DATA_DIR/usr/bin/"
	chmod a+x "$DATA_DIR/usr/bin/baro_temp_cal"
fi

# --- Configuration files ---
mkdir -p "$DATA_DIR/etc/modalai"
for config in voxl-px4-fake-imu-calibration.config voxl-px4-hitl-set-default-parameters.config; do
	if [ -f "$BOARD_DIR/target/$config" ]; then
		cp "$BOARD_DIR/target/$config" "$DATA_DIR/etc/modalai/"
		chmod +x "$DATA_DIR/etc/modalai/$config"
	fi
done

# --- Systemd service ---
mkdir -p "$DATA_DIR/etc/systemd/system"
cp "$PKG_DIR/services/voxl-px4.service" "$DATA_DIR/etc/systemd/system/"

# --- Build metadata ---
mkdir -p "$DATA_DIR/data/px4/etc/extras"
for meta in parameters.json.xz component_general.json.xz actuators.json.xz; do
	if [ -f "$BUILD_APPS/$meta" ]; then
		cp "$BUILD_APPS/$meta" "$DATA_DIR/data/px4/etc/extras/"
	fi
done
if [ -f "$BUILD_APPS/events/all_events.json.xz" ]; then
	cp "$BUILD_APPS/events/all_events.json.xz" "$DATA_DIR/data/px4/etc/extras/"
fi

# --- Create runtime directories ---
mkdir -p "$DATA_DIR/data/px4/param"

################################################################################
# Build the DEB package
################################################################################
mkdir -p "$DEB_DIR"
cp -rf "$PKG_DIR/control/" "$DEB_DIR/DEBIAN"
cp -rf "$DATA_DIR/"*       "$DEB_DIR/"

# Write the version into the control file
sed -i "s/^Version:.*/Version: $VERSION/" "$DEB_DIR/DEBIAN/control"

DEB_NAME="${PACKAGE}_${VERSION}_arm64.deb"
dpkg-deb --build "$DEB_DIR" "$SCRIPT_DIR/$DEB_NAME"

echo ""
echo "Package built: $SCRIPT_DIR/$DEB_NAME"
echo "DONE"
