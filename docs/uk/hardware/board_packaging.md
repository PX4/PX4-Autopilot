# Board Firmware Packaging

PX4 supports building distributable firmware packages for Linux-based (POSIX) boards.
While NuttX boards produce `.px4` firmware files that are flashed via QGroundControl, POSIX boards can produce `.deb` (Debian) packages that are installed using standard Linux package management tools (`dpkg`, `apt`).

This page covers how manufacturers can add `.deb` packaging to their boards, with examples for both single-processor and multi-processor architectures.

## Загальний огляд

The packaging framework uses [CMake CPack](https://cmake.org/cmake/help/latest/module/CPack.html) with the DEB generator.
It is built on two extension points in the PX4 build system:

- **`boards/<vendor>/<board>/cmake/package.cmake`**: CPack variable overrides (package name, version, dependencies, architecture, maintainer info). Loaded during CMake configure.
- **`boards/<vendor>/<board>/cmake/install.cmake`**: `install()` rules that define what goes into the package (binaries, scripts, config files, service files). Loaded from `platforms/posix/CMakeLists.txt` where build targets are available.

When a board provides these files, CI automatically discovers and builds the `_deb` target alongside the normal firmware build.

## Build Command

For any board with packaging support:

```sh
make <vendor>_<board>_deb
```

Наприклад:

```sh
make modalai_voxl2_deb
```

This builds the `_default` configuration (and any companion builds for multi-processor boards), then runs `cpack -G DEB` in the build directory.
The resulting `.deb` file is placed in `build/<vendor>_<board>_default/`.

## Adding Packaging to a Board

### File Structure

```
boards/<vendor>/<board>/
    cmake/
        package.cmake      # CPack configuration (required)
        install.cmake      # Install rules (required)
    debian/
        postinst           # Post-install script (optional)
        prerm              # Pre-remove script (optional)
        <name>.service     # Systemd unit file (optional)
```

### Step 1: CPack Configuration (package.cmake)

This file sets CPack variables that control the `.deb` metadata.
It is included from `cmake/package.cmake` after the base CPack defaults are configured.

```cmake
# boards/<vendor>/<board>/cmake/package.cmake

# Derive Debian-compatible version from git tag
# v1.17.0-alpha1-42-gabcdef -> 1.17.0~alpha1.42.gabcdef
# v1.17.0 -> 1.17.0
string(REGEX REPLACE "^v" "" _deb_ver "${PX4_GIT_TAG}")
string(REGEX REPLACE "-" "~" _deb_ver "${_deb_ver}")
string(REGEX REPLACE "~([0-9]+)~" ".\\1." _deb_ver "${_deb_ver}")

# Target architecture (use the target arch, not the build host)
set(CPACK_DEBIAN_ARCHITECTURE "arm64")

# Package identity
set(CPACK_DEBIAN_PACKAGE_NAME "my-px4-board")
set(CPACK_DEBIAN_FILE_NAME "my-px4-board_${_deb_ver}_arm64.deb")

# Install prefix
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")
set(CPACK_INSTALL_PREFIX "/usr")
set(CPACK_SET_DESTDIR true)

# Package metadata
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "PX4 Autopilot for My Board")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Vendor <support@vendor.com>")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "some-dependency (>= 1.0)")
set(CPACK_DEBIAN_PACKAGE_CONFLICTS "")
set(CPACK_DEBIAN_PACKAGE_REPLACES "")

# Disable shlibdeps for cross-compiled boards
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)

# Include post-install and pre-remove scripts (optional)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${PX4_BOARD_DIR}/debian/postinst;${PX4_BOARD_DIR}/debian/prerm")
```

**Key variables:**

| Змінні                           | Ціль                                                                                                                                                                    |
| -------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `CPACK_DEBIAN_ARCHITECTURE`      | Target architecture. Set explicitly for cross-compiled boards since `dpkg --print-architecture` reports the build host, not the target. |
| `CPACK_DEBIAN_PACKAGE_NAME`      | Package name as it appears in `dpkg -l`.                                                                                                                |
| `CPACK_DEBIAN_FILE_NAME`         | Output `.deb` filename.                                                                                                                                 |
| `CPACK_DEBIAN_PACKAGE_DEPENDS`   | Runtime dependencies (comma-separated, Debian format).                                                                               |
| `CPACK_DEBIAN_PACKAGE_SHLIBDEPS` | Set to `OFF` for cross-compiled boards where `dpkg-shlibdeps` cannot inspect target binaries.                                                           |

### Step 2: Install Rules (install.cmake)

This file defines what files are packaged in the `.deb`.
It is included from `platforms/posix/CMakeLists.txt` where the `px4` build target is available.

All paths are relative to `CPACK_PACKAGING_INSTALL_PREFIX` (typically `/usr`). Use `../` to install outside the prefix (e.g., `../etc/` installs to `/etc/`).

**Minimal example (single-processor board):**

```cmake
# boards/<vendor>/<board>/cmake/install.cmake

# PX4 binary
install(TARGETS px4 RUNTIME DESTINATION bin)

# Module alias script (generated during build)
install(PROGRAMS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/px4-alias.sh DESTINATION bin)

# Startup scripts
install(PROGRAMS
    ${PX4_BOARD_DIR}/target/my-px4-start
    DESTINATION bin
)

# Configuration files
install(FILES
    ${PX4_BOARD_DIR}/target/my-config.conf
    DESTINATION ../etc/my-board
)

# Systemd service
install(FILES ${PX4_BOARD_DIR}/debian/my-px4.service
    DESTINATION ../etc/systemd/system
)

# Component metadata
install(FILES
    ${PX4_BINARY_DIR}/actuators.json.xz
    ${PX4_BINARY_DIR}/parameters.json.xz
    DESTINATION ../data/px4/etc/extras
    OPTIONAL
)
install(FILES ${PX4_BINARY_DIR}/events/all_events.json.xz
    DESTINATION ../data/px4/etc/extras
    OPTIONAL
)
```

### Step 3: Debian Scripts (optional)

#### postinst

Runs after the package is installed. Common tasks:

- Create `px4-*` module symlinks from `px4-alias.sh`
- Set up required directories with correct ownership
- Run `systemctl daemon-reload` to pick up the service file
- Board-specific setup (e.g., DSP signature generation)

```bash
#!/bin/bash
set -e

# Create px4-* symlinks
if [ -f /usr/bin/px4-alias.sh ]; then
    grep "^alias " /usr/bin/px4-alias.sh | \
        sed "s/alias \(px4-[a-zA-Z0-9_]*\)=.*/\1/" | while read cmd; do
        ln -sf px4 "/usr/bin/${cmd}"
    done
fi

# Create data directories
mkdir -p /data/px4/param
mkdir -p /data/px4/etc/extras

# Reload systemd
if command -v systemctl > /dev/null 2>&1; then
    systemctl daemon-reload
fi
```

#### prerm

Runs before the package is removed:

```bash
#!/bin/bash
set -e

# Stop the service
if command -v systemctl > /dev/null 2>&1; then
    systemctl stop my-px4 2>/dev/null || true
fi

# Remove px4-* symlinks
for f in /usr/bin/px4-*; do
    if [ -L "$f" ] && [ "$(readlink "$f")" = "px4" ]; then
        rm -f "$f"
    fi
done
```

Both scripts must be executable (`chmod +x`).

## Multi-Processor Boards

Some boards run PX4 across multiple processors, for example the ModalAI VOXL2 which has a POSIX apps processor (ARM) and a Hexagon DSP (SLPI).
These produce two separate CMake builds, but the `.deb` must contain artifacts from both.

### How It Works

1. The `_default` build (POSIX/apps processor) owns the `.deb`.
2. The Makefile `%_deb` target builds `_default`, which chains any companion builds as CMake prerequisites.
3. The `install.cmake` pulls companion build artifacts via absolute path to the sibling build directory.
4. CPack runs in the `_default` build tree and produces a single `.deb`.

### Companion Build Artifacts

In `install.cmake`, reference the companion build output by absolute path:

```cmake
# DSP firmware blob from companion SLPI build
set(SLPI_BUILD_DIR "${PX4_SOURCE_DIR}/build/<vendor>_<board>-slpi_default")

install(FILES ${SLPI_BUILD_DIR}/platforms/qurt/libpx4.so
    DESTINATION lib/rfsa/adsp
    OPTIONAL
)
```

The `OPTIONAL` keyword allows the `.deb` to build even when the companion build hasn't run (useful for development/testing of just the apps-processor side).

### VOXL2 Reference

The VOXL2 board is a complete working example of multi-processor packaging:

```
boards/modalai/voxl2/
    cmake/
        package.cmake      # CPack config: voxl-px4, arm64, deps, shlibdeps off
        install.cmake      # px4 binary, SLPI libpx4.so, scripts, configs, metadata
    debian/
        postinst           # Symlinks, DSP signature, directory setup
        prerm              # Stop service, remove symlinks
        voxl-px4.service   # Systemd unit (after sscrpcd, restart on-failure)
    target/
        voxl-px4           # Main startup wrapper
        voxl-px4-start     # PX4 module startup script
```

The resulting `.deb` installs:

| Path                                   | Contents                                          |
| -------------------------------------- | ------------------------------------------------- |
| `/usr/bin/px4`                         | Apps processor PX4 binary                         |
| `/usr/bin/px4-alias.sh`                | Module alias script                               |
| `/usr/bin/voxl-px4`                    | Startup wrapper                                   |
| `/usr/bin/voxl-px4-start`              | Module startup script                             |
| `/usr/lib/rfsa/adsp/libpx4.so`         | DSP firmware (from SLPI build) |
| `/etc/modalai/*.config`                | Board configuration files                         |
| `/etc/systemd/system/voxl-px4.service` | Systemd service                                   |
| `/data/px4/etc/extras/*.json.xz`       | Component metadata                                |

## CI Integration

### Automatic Discovery

The CI system (`Tools/ci/generate_board_targets_json.py`) automatically discovers boards with `cmake/package.cmake` and adds a `<vendor>_<board>_deb` target to the board's existing CI group.
No manual CI configuration is needed.

### Artifact Collection

The `Tools/ci/package_build_artifacts.sh` script collects `.deb` files alongside `.px4` and `.elf` artifacts.
On tagged releases, `.deb` files are uploaded to both S3 and GitHub Releases.

## Version Format

The `.deb` version is derived from `PX4_GIT_TAG` using Debian-compatible formatting:

| Git Tag                     | Debian Version             | Примітки                                                  |
| --------------------------- | -------------------------- | --------------------------------------------------------- |
| `v1.17.0`                   | `1.17.0`                   | Stable release                                            |
| `v1.17.0-beta1`             | `1.17.0~beta1`             | Pre-release (`~` sorts before release) |
| `v1.17.0-alpha1-42-gabcdef` | `1.17.0~alpha1.42.gabcdef` | Development build                                         |

The `~` prefix in Debian versioning ensures pre-releases sort lower than the final release: `1.17.0~beta1 < 1.17.0`.

## Checklist for New Boards

1. Create `boards/<vendor>/<board>/cmake/package.cmake` with CPack variables
2. Create `boards/<vendor>/<board>/cmake/install.cmake` with install rules
3. (Optional) Create `boards/<vendor>/<board>/debian/postinst` and `prerm`
4. (Optional) Create `boards/<vendor>/<board>/debian/<name>.service`
5. Test locally: `make <vendor>_<board>_deb`
6. Verify: `dpkg-deb --info build/<vendor>_<board>_default/<name>_*.deb`
7. Verify: `dpkg-deb --contents build/<vendor>_<board>_default/<name>_*.deb`
8. CI picks it up automatically on the next push
