# PX4 SITL .deb Package

PX4 provides a pre-built `.deb` package for running SITL (Software-In-The-Loop) simulation without building from source.

## Package

| Package | Contents |
|---------|----------|
| `px4-sitl` | PX4 binary, ROMFS, init scripts, wrapper script, Gazebo Harmonic models, worlds, and plugins |

## Installation

The package depends on Gazebo Harmonic runtime libraries from the OSRF repository.
Add the repository first, then install the `.deb`:

```bash
# Add OSRF Gazebo repository (one-time setup)
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

# Install (resolves Gazebo dependencies automatically)
sudo apt install ./px4-sitl_*.deb
```

## Usage

### SIH Simulation (no external simulator needed)

```bash
PX4_SIM_MODEL=sihsim_quadx px4-sitl
```

### Gazebo Simulation

```bash
PX4_SIM_MODEL=gz_x500 px4-sitl
```

### Daemon Mode (for CI)

```bash
PX4_SIM_MODEL=sihsim_quadx px4-sitl -d
```

### Multi-Instance

```bash
PX4_SIM_MODEL=gz_x500 px4-sitl -i 0
PX4_SIM_MODEL=gz_x500 px4-sitl -i 1
```

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `PX4_SIM_MODEL` | Simulation model to use (e.g., `gz_x500`, `sihsim_quadx`) | (none) |
| `PX4_GZ_WORLD` | Gazebo world file (without `.sdf` extension) | `default` |
| `HEADLESS` | Set to `1` to disable Gazebo GUI | (unset) |
| `XDG_STATE_HOME` | Base directory for logs | `$HOME/.local/state` |
| `XDG_DATA_HOME` | Base directory for working data | `$HOME/.local/share` |

## Installed File Layout

```
/opt/px4-sitl/
  bin/
    px4           # PX4 binary
    px4-sitl      # Wrapper script
    px4-*         # Module symlinks
    px4-alias.sh  # Shell aliases
  etc/            # ROMFS (init scripts, mixers, airframes)
    init.d-posix/
      rcS
      px4-rc.gzsim
      ...
  share/gz/
    models/
    worlds/
    server.config
  lib/gz/plugins/
    *.so

/usr/bin/px4-sitl -> /opt/px4-sitl/bin/px4-sitl  # symlink

# Runtime directories (per-user, created on first run):
$XDG_DATA_HOME/px4/rootfs/<instance>/   # working directory
$XDG_STATE_HOME/px4/logs/               # flight logs
```

## Building Locally

```bash
make px4_sitl_default
cd build/px4_sitl_default
cpack -G DEB
```

This produces a single `px4-sitl_*.deb` file in the build directory.

## Uninstallation

```bash
sudo apt remove px4-sitl
```
