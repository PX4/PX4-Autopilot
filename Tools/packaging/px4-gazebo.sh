#!/usr/bin/env bash
# px4-gazebo: Launch PX4 SITL with Gazebo from the installed .deb package
set -e

PX4_GAZEBO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PX4_BINARY="${PX4_GAZEBO_DIR}/bin/px4"

# Set Gazebo resource paths so gz-sim finds PX4 models, worlds, and plugins.
export PX4_GZ_MODELS="${PX4_GAZEBO_DIR}/share/gz/models"
export PX4_GZ_WORLDS="${PX4_GAZEBO_DIR}/share/gz/worlds"
export PX4_GZ_PLUGINS="${PX4_GAZEBO_DIR}/lib/gz/plugins"
export PX4_GZ_SERVER_CONFIG="${PX4_GAZEBO_DIR}/share/gz/server.config"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:${PX4_GZ_PLUGINS}"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_GZ_SERVER_CONFIG}"

# Gazebo's Physics system searches for "gz-physics-dartsim-plugin" which maps
# to the unversioned libgz-physics-dartsim-plugin.so. The runtime package only
# ships versioned .so files; the unversioned symlink lives in the -dev package.
# Create it if missing so Gazebo finds the DART engine without installing -dev.
GZ_PHYSICS_ENGINE_DIR=$(echo /usr/lib/*/gz-physics-7/engine-plugins)
if [ -d "$GZ_PHYSICS_ENGINE_DIR" ]; then
    UNVERSIONED="$GZ_PHYSICS_ENGINE_DIR/libgz-physics-dartsim-plugin.so"
    if [ ! -e "$UNVERSIONED" ]; then
        VERSIONED=$(ls "$GZ_PHYSICS_ENGINE_DIR"/libgz-physics*-dartsim-plugin.so.* 2>/dev/null | head -1)
        if [ -n "$VERSIONED" ]; then
            ln -sf "$(basename "$VERSIONED")" "$UNVERSIONED" 2>/dev/null || true
        fi
    fi
fi

exec "${PX4_BINARY}" "$@"
