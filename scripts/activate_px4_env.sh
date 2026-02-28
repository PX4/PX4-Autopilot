#!/usr/bin/env bash
# Activate PX4 build environment: virtualenv + Homebrew/Gazebo/Qt flags

# Stop any conda env that might interfere
conda deactivate >/dev/null 2>&1 || true

# Activate project virtualenv (assumes repo root)
if [ -f "$(pwd)/.venv_kconfig/bin/activate" ]; then
  # shellcheck disable=SC1090
  source "$(pwd)/.venv_kconfig/bin/activate"
else
  echo "Warning: .venv_kconfig not found in $(pwd). Run Tools/setup or create the venv first."
fi

# Homebrew pkg-config and Qt5
export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:/opt/homebrew/share/pkgconfig:${PKG_CONFIG_PATH:-}"
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5/lib/cmake:${CMAKE_PREFIX_PATH:-}"

# Homebrew include/lib flags (adjust gstreamer version if needed)
export LDFLAGS="-L/opt/homebrew/lib -L/opt/homebrew/Cellar/gstreamer/1.28.1/lib ${LDFLAGS:-}"
export CPPFLAGS="-I/opt/homebrew/include -I/opt/homebrew/Cellar/gstreamer/1.28.1/include/gstreamer-1.0 ${CPPFLAGS:-}"

# Runtime loader fallback so plugins find build and Homebrew libs
export DYLD_FALLBACK_LIBRARY_PATH="$(pwd)/build/px4_sitl_default/external/Install/lib:$(pwd)/build/px4_sitl_default/OpticalFlow/install/lib:/opt/homebrew/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"

# Ensure Homebrew binaries are first
export PATH="/opt/homebrew/bin:$PATH"

echo "PX4 env activated: venv (if present), PKG_CONFIG_PATH, CMAKE_PREFIX_PATH, LDFLAGS, CPPFLAGS, DYLD_FALLBACK_LIBRARY_PATH set."#!/usr/bin/env bash
# Activate PX4 build env: venv + Homebrew/Gazebo/Qt flags

# stop any conda env interfering
conda deactivate >/dev/null 2>&1 || true

# activate venv
source .venv_kconfig/bin/activate

# Homebrew pkg-config + Qt5
export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:/opt/homebrew/share/pkgconfig:$PKG_CONFIG_PATH"
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5/lib/cmake:${CMAKE_PREFIX_PATH:-}"

# Homebrew include/lib flags
export LDFLAGS="-L/opt/homebrew/lib -L/opt/homebrew/Cellar/gstreamer/1.28.1/lib ${LDFLAGS:-}"
export CPPFLAGS="-I/opt/homebrew/include -I/opt/homebrew/Cellar/gstreamer/1.28.1/include/gstreamer-1.0 ${CPPFLAGS:-}"

# runtime loader fallback (so plugins find build and Homebrew libs)
export DYLD_FALLBACK_LIBRARY_PATH="$PWD/build/px4_sitl_default/external/Install/lib:$PWD/build/px4_sitl_default/OpticalFlow/install/lib:/opt/homebrew/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"

# ensure Homebrew bin first
export PATH="/opt/homebrew/bin:$PATH"

echo "Activated .venv_kconfig and PX4 env (pkg-config, LDFLAGS, CPPFLAGS, DYLD_FALLBACK_LIBRARY_PATH)."
