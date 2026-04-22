# Hawkeye Visualizer

[Hawkeye](https://px4.github.io/Hawkeye/) is a real-time 3D flight _visualizer_ for PX4.

Hawkeye is the natural pair for [SIH](../sim_sih/index.md) — SIH runs the physics of an aircraft simulation and outputs MAVLink [HIL_STATE_QUATERNION](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) messages, Hawkeye uses these to show you what's happening.

Hawkeye has zero runtime dependencies, supports up to 16 vehicles simultaneously, and can replay PX4 ULog (`.ulg`) flight logs with transport controls, markers, and multi-drone correlation analysis.

## Install

### macOS (Homebrew)

```sh
brew tap PX4/px4
brew install PX4/px4/hawkeye
```

### Linux (Debian/Ubuntu)

Download the `.deb` from the [Hawkeye releases page](https://github.com/PX4/Hawkeye/releases/latest):

```sh
sudo dpkg -i hawkeye-*.deb
```

### Windows and source builds

For Ubuntu 24.04 or later in WSL2 you can install the packages in the same way:

```sh
sudo dpkg -i hawkeye-*.deb
```

For other versions of Ubuntu (or native Windows builds) you may need to [Build from source](https://px4.github.io/Hawkeye/developer/build) (Hawkeye docs).

## Usage with SIH

Start PX4 SIH, then launch Hawkeye in a separate terminal:

```sh
# Terminal 1
make px4_sitl sihsim_quadx

# Terminal 2
hawkeye
```

Hawkeye listens on UDP port 19410 — the same port SIH sends [HIL_STATE_QUATERNION](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) on — so no configuration is needed.
The vehicle appears in the Hawkeye window as soon as SIH starts streaming.

For fixed-wing or tailsitter simulation, Hawkeye auto-detects the vehicle type from MAVLink `HEARTBEAT` and loads the matching 3D model.

## Full documentation

Complete documentation — including multi-vehicle SITL, ULog replay, HUD modes, camera controls, and correlation analysis — lives at **[px4.github.io/Hawkeye](https://px4.github.io/Hawkeye/)**.

- [First SITL run](https://px4.github.io/Hawkeye/first-sitl) — the shortest path from install to seeing a vehicle move
- [Multi-Drone Replay](https://px4.github.io/Hawkeye/multi_drone) — compare multiple flights with deconfliction and correlation
- [Live SITL Integration](https://px4.github.io/Hawkeye/sitl) — single-vehicle and multi-instance swarm workflows
- [Command-Line Reference](https://px4.github.io/Hawkeye/cli) — every CLI flag with examples
- [Source code](https://github.com/PX4/Hawkeye)
