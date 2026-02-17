# PX4 SITL .deb Packages

PX4 provides pre-built `.deb` packages for running SITL (Software-In-The-Loop) simulation without building from source.

## Packages

| Package | Contents | Dependencies |
|---------|----------|--------------|
| `px4-sitl` | PX4 binary, ROMFS, init scripts, wrapper script, Gazebo Harmonic models, worlds, and plugins | `gz-sim8-cli`, `libgz-sim8-plugins`, `libgz-physics7-dartsim`, `gz-tools2` |
| `px4-sitl-sih` | PX4 binary, ROMFS, init scripts, wrapper script (no Gazebo) | `libc6`, `libstdc++6` |

The `px4-sitl-sih` package uses the built-in SIH (Simulation-In-Hardware) physics engine and does not require any external simulator.

## Installation

### px4-sitl (with Gazebo Harmonic)

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

### px4-sitl-sih (no Gazebo)

No extra repositories required:

```bash
sudo apt install ./px4-sitl-sih_*.deb
```

## Usage

### SIH Simulation (no external simulator needed)

Works with either package:

```bash
PX4_SIM_MODEL=sihsim_quadx px4-sitl      # if px4-sitl is installed
PX4_SIM_MODEL=sihsim_quadx px4-sitl-sih  # if px4-sitl-sih is installed
```

### Gazebo Simulation (px4-sitl only)

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

## Communication Interfaces

Both packages expose the same MAVLink and uXRCE-DDS interfaces. Zenoh is not included in either package.

### MAVLink (UDP)

PX4 starts five MAVLink instances by default (four plus one SIH-only link). Port numbers are offset by the instance ID (the `-i` flag) for multi-vehicle setups.

| Link | Mode | UDP Local Port | UDP Remote Port | Data Rate |
|------|------|---------------|----------------|-----------|
| GCS link | Normal | 18570 + instance | 14550 | 4 Mbps |
| API/Offboard link | Onboard | 14580 + instance | 14540 + instance | 4 Mbps |
| Onboard link to camera | Onboard | 14280 + instance | 14030 + instance | 4 kbps |
| Onboard link to gimbal | Gimbal | 13030 + instance | 13280 + instance | 400 kbps |
| SIH display (SIH only) | Custom | 19450 + instance | 19410 + instance | 400 kbps |

By default, MAVLink only listens on localhost. Set parameter `MAV_{i}_BROADCAST = 1` to enable network access.

### uXRCE-DDS (ROS 2)

The `uxrce_dds_client` module starts automatically and connects to a Micro XRCE-DDS Agent over UDP.

| Setting | Default |
|---------|---------|
| Transport | UDP |
| Agent IP | `127.0.0.1` |
| Agent Port | `8888` |

To use with ROS 2, run the Micro XRCE-DDS Agent on the host:

```bash
MicroXRCEAgent udp4 -p 8888
```

The agent port is configurable via the `PX4_UXRCE_DDS_PORT` environment variable, and a ROS namespace can be set with `PX4_UXRCE_DDS_NS`.

### Zenoh

Zenoh is **not available** in either package. It requires the separate `px4_sitl_zenoh` build variant.

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `PX4_SIM_MODEL` | Simulation model to use (e.g., `gz_x500`, `sihsim_quadx`) | (none) |
| `PX4_GZ_WORLD` | Gazebo world file (without `.sdf` extension) | `default` |
| `HEADLESS` | Set to `1` to disable Gazebo GUI | (unset) |
| `PX4_UXRCE_DDS_PORT` | uXRCE-DDS agent UDP port | `8888` |
| `PX4_UXRCE_DDS_NS` | uXRCE-DDS ROS namespace | (none) |
| `XDG_STATE_HOME` | Base directory for logs | `$HOME/.local/state` |
| `XDG_DATA_HOME` | Base directory for working data | `$HOME/.local/share` |

## Installed File Layout

### px4-sitl

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
```

### px4-sitl-sih

```
/opt/px4-sitl-sih/
  bin/
    px4            # PX4 binary
    px4-sitl       # Wrapper script
    px4-*          # Module symlinks
    px4-alias.sh   # Shell aliases
  etc/             # ROMFS (init scripts, mixers, airframes)
    init.d-posix/
      rcS
      ...

/usr/bin/px4-sitl-sih -> /opt/px4-sitl-sih/bin/px4-sitl  # symlink
```

### Runtime directories (per-user, created on first run)

```
$XDG_DATA_HOME/px4/rootfs/<instance>/   # working directory
$XDG_STATE_HOME/px4/logs/               # flight logs
```

## Building Locally

```bash
# Default (with Gazebo)
make px4_sitl_default
cd build/px4_sitl_default && cpack -G DEB

# SIH (no Gazebo)
make px4_sitl_sih
cd build/px4_sitl_sih && cpack -G DEB
```

## Uninstallation

```bash
sudo apt remove px4-sitl       # default package
sudo apt remove px4-sitl-sih   # SIH package
```
