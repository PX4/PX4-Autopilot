# PX4 SITL .deb Packages

PX4 provides pre-built `.deb` packages that let you run a full [SITL](../simulation/index.md) simulation on Ubuntu — no build environment required.
Once installed you have a complete PX4 stack: fly vehicles in simulation, tune parameters, connect a GCS, use MAVLink and uXRCE-DDS, and run multi-instance launches — all without compiling a single line of code.

Two packages are available:

| Package | Simulator | Ubuntu |
|---------|-----------|--------|
| `px4` | [SIH](../sim_sih/index.md) (built-in, no external simulator) | 22.04, 24.04 |
| `px4-gazebo` | [Gazebo Harmonic](../sim_gazebo_gz/index.md) | 22.04, 24.04 |

Packages are built from the `main` branch on every tagged release and are versioned to match PX4 releases (e.g. `1.17.0`).

## Installation

Download the `.deb` file for your Ubuntu version and architecture from the [PX4 Releases](https://github.com/PX4/PX4-Autopilot/releases) page, then install as shown below.

### px4 (SIH, no Gazebo)

No extra repositories required:

```bash
sudo apt install ./px4_*.deb
```

### px4-gazebo (Gazebo Harmonic)

The package depends on Gazebo Harmonic runtime libraries from the OSRF repository.
Add the repository first, then install:

```bash
# Add OSRF Gazebo repository (one-time setup)
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

# Install (resolves Gazebo dependencies automatically)
sudo apt install ./px4-gazebo_*.deb
```

## Usage

### SIH Simulation

Set `PX4_SIM_MODEL` to a [SIH vehicle](../sim_sih/index.md) and run `px4`:

```bash
PX4_SIM_MODEL=sihsim_quadx px4
```

### Gazebo Simulation

Set `PX4_SIM_MODEL` to a Gazebo vehicle and run `px4-gazebo`.
Available models are listed in [Gazebo Vehicles](../sim_gazebo_gz/vehicles.md).

```bash
PX4_SIM_MODEL=gz_x500 px4-gazebo
```

To select a specific [Gazebo world](../sim_gazebo_gz/worlds.md), set `PX4_GZ_WORLD`:

```bash
PX4_GZ_WORLD=arenas PX4_SIM_MODEL=gz_x500 px4-gazebo
```

### Multi-Instance

Multiple simulated vehicle instances can be run simultaneously by passing the `-i` flag with an instance number.
Each instance must be started in a separate terminal:

```bash
# Terminal 1
PX4_SIM_MODEL=gz_x500 px4-gazebo -i 0

# Terminal 2
PX4_SIM_MODEL=gz_x500 px4-gazebo -i 1
```

MAVLink and uXRCE-DDS port numbers are automatically offset by the instance number.
SIH also supports multi-instance with the `px4` package.

Each package (`px4` and `px4-gazebo`) is a standalone install — you cannot mix instances from the two packages in the same simulation session.

### Daemon Mode

Start PX4 without an interactive shell (useful for CI pipelines):

```bash
PX4_SIM_MODEL=sihsim_quadx px4 -d
```

## ROS 2 Integration

The `uxrce_dds_client` module starts automatically and connects to a Micro XRCE-DDS Agent over UDP.
Run the agent before starting PX4:

```bash
MicroXRCEAgent udp4 -p 8888
```

| Setting | Default |
|---------|---------|
| Transport | UDP |
| Agent IP | `127.0.0.1` |
| Agent Port | `8888` |

The environment variables `PX4_UXRCE_DDS_PORT` and `PX4_UXRCE_DDS_NS` override the [UXRCE_DDS_PRT](../advanced_config/parameter_reference.md#UXRCE_DDS_PRT) parameter and ROS namespace at runtime without modifying stored parameters:

```bash
PX4_UXRCE_DDS_PORT=9999 PX4_UXRCE_DDS_NS=drone1 PX4_SIM_MODEL=sihsim_quadx px4
```

## Communication Interfaces

Both packages expose the same MAVLink and uXRCE-DDS interfaces. Zenoh is not included in either package.

### MAVLink UDP Ports

PX4 opens several MAVLink UDP ports on startup. You can connect any MAVLink-compatible tool to these ports — [QGroundControl](https://qgroundcontrol.com), [MAVSDK](https://mavsdk.mavlink.io), or any custom application. Port numbers are offset by the instance number (`-i`).

| Link | Mode | UDP Local Port | UDP Remote Port | Data Rate |
|------|------|---------------|----------------|-----------|
| GCS link | Normal | 18570 + instance | 14550 | 4 Mbps |
| API/Offboard link | Onboard | 14580 + instance | 14540 + instance | 4 Mbps |
| Onboard link to camera | Onboard | 14280 + instance | 14030 + instance | 4 kbps |
| Onboard link to gimbal | Gimbal | 13030 + instance | 13280 + instance | 400 kbps |
| SIH display (SIH only) | Custom | 19450 + instance | 19410 + instance | 400 kbps |

By default, MAVLink only listens on localhost. Set parameter `MAV_{i}_BROADCAST = 1` to enable network access.

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `PX4_SIM_MODEL` | Vehicle model (e.g. `gz_x500`, `sihsim_quadx`) | (required) |
| `PX4_GZ_WORLD` | Gazebo world name, without `.sdf` (e.g. `arenas`) | `default` |
| `HEADLESS` | Set to `1` to disable Gazebo GUI | (unset) |
| `PX4_UXRCE_DDS_PORT` | uXRCE-DDS agent UDP port — overrides [UXRCE_DDS_PRT](../advanced_config/parameter_reference.md#UXRCE_DDS_PRT) | `8888` |
| `PX4_UXRCE_DDS_NS` | uXRCE-DDS ROS namespace — overrides [UXRCE_DDS_PTCFG](../advanced_config/parameter_reference.md#UXRCE_DDS_PTCFG) | (none) |
| `XDG_DATA_HOME` | Base directory for per-instance working data (parameters, dataman) | `$HOME/.local/share` |

## Installed File Layout

### px4

```
/opt/px4/
  bin/
    px4           # PX4 binary
    px4-*         # Module symlinks
    px4-alias.sh  # Shell aliases
  etc/            # ROMFS (init scripts, mixers, airframes)
    init.d-posix/

/usr/bin/px4 -> /opt/px4/bin/px4
```

### px4-gazebo

```
/opt/px4-gazebo/
  bin/
    px4           # PX4 binary
    px4-gazebo    # Gazebo wrapper (sets GZ_SIM_* env vars)
    px4-*         # Module symlinks
    px4-alias.sh  # Shell aliases
  etc/            # ROMFS (init scripts, mixers, airframes)
    init.d-posix/
  share/gz/
    models/       # Gazebo vehicle models
    worlds/       # Gazebo world files
    server.config
  lib/gz/plugins/ # PX4 Gazebo plugins

/usr/bin/px4-gazebo -> /opt/px4-gazebo/bin/px4-gazebo
```

### Runtime directories (created on first run, per user)

```
$XDG_DATA_HOME/px4/rootfs/<instance>/   # parameters, dataman, eeprom
```

## Building Locally

The following commands show how to build the `.deb` files locally, for example to package a custom PX4 branch for deployment:

```bash
# SIH (no Gazebo) — produces px4_*.deb
make px4_sitl_sih
cd build/px4_sitl_sih && cpack -G DEB

# Gazebo — produces px4-gazebo_*.deb
make px4_sitl_default
cd build/px4_sitl_default && cpack -G DEB
```

## Uninstallation

```bash
sudo apt remove px4          # SIH package
sudo apt remove px4-gazebo   # Gazebo package
```
