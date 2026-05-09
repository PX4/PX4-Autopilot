# Pre-built SITL Packages

Pre-built packages let you run [PX4 SITL simulation](index.md) without setting up a build environment.

This is very useful if you don't need to modify PX4 itself.
For example, if you want to write drone apps using [MAVSDK](https://mavsdk.mavlink.io) or [ROS 2](../ros2/user_guide.md), or you just want to fly with PX4.

:::tip
See [PX4 Simulation QuickStart](px4_simulation_quickstart.md) for a one-line instruction to run the SIH package in a container.
:::

## What's Available

Two simulators are packaged, each available as a `.deb` package (Ubuntu) or a Docker [container](#container-images) (any OS):

| 仿真器                                          | Format               | Package / Image         | Size                    |
| -------------------------------------------- | -------------------- | ----------------------- | ----------------------- |
| [SIH](../sim_sih/index.md)                   | .deb | `px4`                   | ~10 MB  |
|                                              | container            | `px4io/px4-sitl`        | ~100 MB |
| [Gazebo Harmonic](../sim_gazebo_gz/index.md) | .deb | `px4-gazebo`            | ~30 MB  |
|                                              | container            | `px4io/px4-sitl-gazebo` | ~2 GB   |

SIH is a lightweight, headless simulator built into PX4 with no external dependencies.
Gazebo provides full 3D simulation with camera, LiDAR, and custom worlds.
Sizes are approximate and vary between releases.

For help choosing between simulators, see the [simulator comparison table](index.md#simulator-comparison).

### Versions and Releases

Packages and images are versioned to match PX4 tags (e.g. `v1.17.0`, `v1.17.0~beta1`).
`.deb` packages are built for **Ubuntu 22.04 (Jammy)** and **24.04 (Noble)**, on both **amd64** and **arm64**.
Container images support **amd64** and **arm64**.
Stable releases and pre-releases are published on the [PX4 Releases](https://github.com/PX4/PX4-Autopilot/releases) page.

## .deb Packages (Ubuntu)

Download the `.deb` file for your Ubuntu version and architecture from the [PX4 Releases](https://github.com/PX4/PX4-Autopilot/releases) page, then install as shown below.
After installation the binary is added to the default Ubuntu system paths, and can be run from anywhere.

### px4 (SIH)

No extra repositories are required:

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

### Uninstalling

```bash
sudo apt remove px4          # SIH package
sudo apt remove px4-gazebo   # Gazebo package
```

## Container Images

Container images are built using the same `.deb` packages described above, packaged into minimal Docker images.
They are published to [Docker Hub](https://hub.docker.com/u/px4io) on every tagged release.
You will need to [install Docker](https://docs.docker.com/get-docker/).

| Image                         | 仿真器                               |
| ----------------------------- | --------------------------------- |
| `px4io/px4-sitl:<tag>`        | SIH (headless) |
| `px4io/px4-sitl-gazebo:<tag>` | Gazebo Harmonic                   |

Tags follow PX4 versions (e.g. `v1.17.0`).

### Running

```bash
# SIH
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl:latest

# Gazebo
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl-gazebo:latest
```

Pass environment variables with `-e`:

```bash
docker run --rm -it -p 14550:14550/udp \
  -e PX4_SIM_MODEL=sihsim_airplane \
  px4io/px4-sitl:latest
```

The quick-start command above only exposes the QGroundControl port.
To use MAVSDK, uXRCE-DDS (ROS 2), or MAVSim Viewer, expose the additional ports:

```bash
docker run --rm -it \
  -p 14550:14550/udp \
  -p 14540:14540/udp \
  -p 8888:8888/udp \
  -p 19410:19410/udp \
  px4io/px4-sitl:latest
```

| Port  | Protocol | Used by                                        |
| ----- | -------- | ---------------------------------------------- |
| 14550 | UDP      | QGroundControl                                 |
| 14540 | UDP      | MAVSDK / offboard API                          |
| 8888  | UDP      | uXRCE-DDS agent (ROS 2)     |
| 19410 | UDP      | SIH display (MAVSim Viewer) |

On Linux, you can skip individual port flags and use `--network host` instead:

```bash
docker run --rm -it --network host px4io/px4-sitl:latest
```

## 配置

These options apply to both `.deb` packages and containers.
Note that after the first section below we only show how to use them with the deb packages (the pattern for using the options doesn't change).

### Vehicle Selection

Set `PX4_SIM_MODEL` to choose a vehicle.

SIH:

```bash
# Deb package
PX4_SIM_MODEL=sihsim_airplane px4

# Container
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl:latest -e PX4_SIM_MODEL=sihsim_airplane
```

Gazebo:

```
# Deb package
PX4_SIM_MODEL=gz_x500 px4-gazebo

# Container
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl-gazebo:latest -e PX4_SIM_MODEL=gz_x500
```

See [SIH Supported Vehicles](../sim_sih/index.md#supported-vehicle-types) and [Gazebo Vehicles](../sim_gazebo_gz/vehicles.md) for the full lists.

### World Selection (Gazebo only)

```sh
PX4_GZ_WORLD=baylands PX4_SIM_MODEL=gz_x500 px4-gazebo
```

See [Gazebo Worlds](../sim_gazebo_gz/worlds.md) for available worlds.

### Environment Variables

| Variable             | 描述                                                                                                     | 默认值                           |
| -------------------- | ------------------------------------------------------------------------------------------------------ | ----------------------------- |
| `PX4_SIM_MODEL`      | Vehicle model (e.g. `gz_x500`, `sihsim_quadx`)      | (required) |
| `PX4_GZ_WORLD`       | Gazebo world name, without `.sdf` (e.g. `baylands`) | `default`                     |
| `HEADLESS`           | Set to `1` to disable Gazebo GUI                                                                       | (unset)    |
| `PX4_UXRCE_DDS_PORT` | uXRCE-DDS agent UDP port                                                                               | `8888`                        |
| `PX4_UXRCE_DDS_NS`   | uXRCE-DDS ROS namespace                                                                                | (none)     |
| `XDG_DATA_HOME`      | Base directory for per-instance working data (parameters, dataman)                  | `$HOME/.local/share`          |

## Multi-Instance

Multiple simulated vehicles can run simultaneously by passing the `-i` flag with an instance number.
Each instance must be started in a separate terminal (or container). This works with both simulators.

```sh
# Terminal 1
PX4_SIM_MODEL=sihsim_quadx px4 -i 0

# Terminal 2
PX4_SIM_MODEL=sihsim_quadx px4 -i 1
```

MAVLink and uXRCE-DDS port numbers are automatically offset by the instance number.

Each package (`px4` and `px4-gazebo`) is a standalone install. Do not mix instances from the two packages in the same session.

## MAVLink and QGroundControl

PX4 opens several MAVLink UDP ports on startup.
[QGroundControl](https://qgroundcontrol.com) auto-connects on UDP port 14550.
You can also connect [MAVSDK](https://mavsdk.mavlink.io) or any MAVLink-compatible tool.

| Link                                      | 模式      | UDP Local Port   | UDP Remote Port  | Data Rate |
| ----------------------------------------- | ------- | ---------------- | ---------------- | --------- |
| GCS link                                  | Normal  | 18570 + instance | 14550            | 4 Mbps    |
| API/Offboard link                         | Onboard | 14580 + instance | 14540 + instance | 4 Mbps    |
| Onboard link to camera                    | Onboard | 14280 + instance | 14030 + instance | 4 kbps    |
| Onboard link to gimbal                    | Gimbal  | 13030 + instance | 13280 + instance | 400 kbps  |
| SIH display (SIH only) | Custom  | 19450 + instance | 19410 + instance | 400 kbps  |

By default, MAVLink only listens on localhost.
Set parameter `MAV_{i}_BROADCAST = 1` to enable network access.

## ROS 2 Integration

The `uxrce_dds_client` module starts automatically and connects to a Micro XRCE-DDS Agent over UDP.
Run the agent before starting PX4:

```sh
MicroXRCEAgent udp4 -p 8888
```

| 设置         | 默认值         |
| ---------- | ----------- |
| Transport  | UDP         |
| Agent IP   | `127.0.0.1` |
| Agent Port | `8888`      |

Environment variables `PX4_UXRCE_DDS_PORT` and `PX4_UXRCE_DDS_NS` override the corresponding PX4 parameters ([UXRCE_DDS_PRT](../advanced_config/parameter_reference.md#UXRCE_DDS_PRT), [UXRCE_DDS_NS_IDX](../advanced_config/parameter_reference.md#UXRCE_DDS_NS_IDX)) at runtime without modifying stored parameters:

```sh
PX4_UXRCE_DDS_PORT=9999 PX4_UXRCE_DDS_NS=drone1 PX4_SIM_MODEL=sihsim_quadx px4
```

## Daemon Mode

Start PX4 without an interactive shell (useful for CI pipelines and automated testing):

```sh
PX4_SIM_MODEL=sihsim_quadx px4 -d
```

## Installed File Layout

### px4

```txt
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

```txt
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

```sh
$XDG_DATA_HOME/px4/rootfs/<instance>/   # parameters, dataman, eeprom
```

## Building .deb Files Locally

To build `.deb` files locally (e.g. to package a custom PX4 branch):

```sh
# SIH — produces px4_*.deb
make px4_sitl_sih
cd build/px4_sitl_sih && cpack -G DEB

# Gazebo — produces px4-gazebo_*.deb
make px4_sitl_default
cd build/px4_sitl_default && cpack -G DEB
```
