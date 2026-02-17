#!/usr/bin/env bash
# px4-sitl: Launch PX4 SITL from the installed .deb package
set -e

PX4_SITL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PX4_BINARY="${PX4_SITL_DIR}/bin/px4"

# XDG-compliant state directory (matches AppImage PR #25105 patterns)
STATE_HOME="${XDG_STATE_HOME:-$HOME/.local/state}"
DATA_HOME="${XDG_DATA_HOME:-$HOME/.local/share}"
PX4_STATE_DIR="${STATE_HOME}/px4"
PX4_DATA_DIR="${DATA_HOME}/px4"

# Parse instance number for multi-vehicle support
INSTANCE=0
prev_was_i=0
for arg in "$@"; do
    if [ "$prev_was_i" = "1" ]; then INSTANCE="$arg"; prev_was_i=0; fi
    if [ "$arg" = "-i" ]; then prev_was_i=1; fi
done

# Working directory for runtime state (parameters, dataman, eeprom)
WORKDIR="${PX4_DATA_DIR}/rootfs/${INSTANCE}"
LOG_DIR="${PX4_STATE_DIR}/logs"
mkdir -p "${WORKDIR}" "${LOG_DIR}" "${PX4_STATE_DIR}"

# Generate gz_env.sh for Gazebo if the gazebo package is installed
# (px4-rc.gzsim sources ./gz_env.sh from the working directory)
if [ -d "${PX4_SITL_DIR}/share/gz/models" ]; then
    cat > "${WORKDIR}/gz_env.sh" <<GZEOF
export PX4_GZ_MODELS=${PX4_SITL_DIR}/share/gz/models
export PX4_GZ_WORLDS=${PX4_SITL_DIR}/share/gz/worlds
export PX4_GZ_PLUGINS=${PX4_SITL_DIR}/lib/gz/plugins
export PX4_GZ_SERVER_CONFIG=${PX4_SITL_DIR}/share/gz/server.config
export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$PX4_GZ_MODELS:\$PX4_GZ_WORLDS
export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:\$PX4_GZ_PLUGINS
export GZ_SIM_SERVER_CONFIG_PATH=\$PX4_GZ_SERVER_CONFIG
GZEOF

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
fi

export PATH="${PX4_SITL_DIR}/bin:${WORKDIR}/bin:${PATH}"
cd "${WORKDIR}"
exec "${PX4_BINARY}" "${PX4_SITL_DIR}/etc" -w "${WORKDIR}" "$@"
