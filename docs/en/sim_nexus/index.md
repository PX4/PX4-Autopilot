# Nexus Simulation

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

Built on [Newton](https://github.com/newton-physics/newton), [Warp](https://github.com/NVIDIA/warp) and [OpenUSD](https://openusd.org/), [nexus](https://breuerpeter.github.io/nexus/) lets you develop drone subsystems and test them as part of the full flight stack.
Newton is an open-source physics engine, Warp is the Python framework that compiles its kernels for the GPU or the CPU, and OpenUSD is the scene description format that holds the vehicle and the scene.

PX4 flies the simulated vehicle as [SITL](../simulation/index.md#sitl-simulation-environment), in lockstep over the [Simulator MAVLink API](../simulation/index.md#simulator-mavlink-api) on TCP port `4560`.
One command starts both: it fetches and builds PX4, runs it in a Docker container beside the simulation, and stops it on exit.

The project is open source (Apache 2.0) and installs from [PyPI](https://pypi.org/project/nexus-uav/).
You can watch a [recorded PX4 flight](https://breuerpeter.github.io/nexus/examples/px4-sitl/) in the browser.

## Why Use Nexus?

- PX4 and the simulation start and stop together, so a flight takes one command and one terminal.
- The vehicle is one OpenUSD file that declares its sensors, ESCs, motors and propellers, and you can [fly a vehicle of your own](https://breuerpeter.github.io/nexus/guide/your-own-assets/).
- A run can write one [Rerun](https://rerun.io) recording that holds the simulation's ground truth and, from PX4's ULog, what PX4 estimated.
  See [Observability in Rerun](https://breuerpeter.github.io/nexus/guide/running/#observability-in-rerun).
- A script can command PX4 over MAVLink for unattended test flights, with the project's [offboard client](https://breuerpeter.github.io/nexus/reference/api/px4/).
  The project's own CI flies PX4 this way, and fails on any PX4 warning about the simulation.
- A vehicle that declares a camera or a lidar renders it with RTX.
  See [RTX cameras and lidar](https://breuerpeter.github.io/nexus/guide/running/#rtx-cameras-and-lidar).
- The same vehicle file trains a policy in Isaac Lab, and the design examples take gradients through Newton from it.
  See the [examples](https://breuerpeter.github.io/nexus/examples/).

## Requirements

- Linux
- A CUDA-capable NVIDIA GPU
- [Docker](https://docs.docker.com/engine/install/)
- [uv](https://docs.astral.sh/uv/getting-started/installation/), which fetches Python 3.12 if the machine lacks it
- [QGroundControl](https://qgroundcontrol.com), to fly by hand

You do not need a PX4 checkout or the PX4 toolchain: the PX4 build runs in a container.

## Installation

Install the package from PyPI into a new environment:

```sh
uv venv --python 3.12
uv pip install nexus-uav
```

## Running the Simulation

1. Start QGroundControl.
   It listens on UDP port `14550` and connects when PX4 sends its first heartbeat.

2. Start the simulation with a vehicle and a scene:

   ```sh
   uv run nexus run --vehicle astro_max_base --scene empty
   ```

   The first run on a machine fetches the PX4 commit that the release pins into `~/.cache/nexus/px4_sitl/`, and builds it there.
   This takes some minutes once.
   Later runs rebuild only what changed.

3. Wait for QGroundControl to connect, then arm and fly.
   PX4's console is the file `px4_sitl/console.log` in the run's folder, `~/.cache/nexus/logs/<stamp>-<pid>/`.
   PX4 writes its `.ulg` flight log to the same folder.

4. Stop the simulation with `Ctrl+C`.
   The run stops the PX4 container too.

To watch the flight live, add `--view` to the command and connect the Rerun viewer from a second terminal:

```sh
uv run rerun --connect
```

To write the recording to a file, add `--log` in place of `--view`.

PX4 keeps its parameters and its last mission across runs, as a vehicle keeps them across a reboot.
Add `--clean` to start from the airframe's defaults.

## Using Your Own PX4 Checkout

To fly a PX4 tree that you work on, set `NEXUS_PX4_SITL_REF` to the absolute path of the checkout:

```sh
NEXUS_PX4_SITL_REF=/home/user/PX4-Autopilot uv run nexus run --vehicle astro_max_base --scene empty
```

The run builds that checkout in place and flies it.
It fetches nothing.

The checkout must hold the vehicle's airframe, `none_astro_max`.
A tree without it stops the run with `ERROR [init] Unknown model none_astro_max`.

You can also start PX4 yourself, for example under a debugger.
A small OpenUSD layer drops PX4 from the run, and the run then waits for PX4 on TCP port `4560`.
See [PX4 as a peer](https://breuerpeter.github.io/nexus/guide/running/#px4-as-a-peer).

## Ports

PX4 numbers its links from its instance `N`.
A run takes the lowest instance that is free on the machine, so two runs do not collide.

| Port        | Protocol | Link                                                  |
| ----------- | -------- | ----------------------------------------------------- |
| `4560 + N`  | TCP      | Simulator MAVLink API: sensors to PX4, actuators back |
| `14540 + N` | UDP      | Offboard MAVLink link, for scripts                    |
| `14550`     | UDP      | QGroundControl                                        |
| `9876`      | gRPC     | Rerun recording, with `--view`                        |

## Supported Vehicles

| Vehicle                                   | Simulator vehicle | PX4 airframe                               |
| ----------------------------------------- | ----------------- | ------------------------------------------ |
| Freefly Systems Astro Max                 | `astro_max_base`  | `none_astro_max` (`SYS_AUTOSTART` `80000`) |
| Freefly Systems Astro Max with FPV camera | `astro_max_fpv`   | `none_astro_max` (`SYS_AUTOSTART` `80000`) |

The FPV camera needs an NVIDIA GPU with RTX and the extra `nexus-uav[rtx]`.

The shipped scenes are `empty`, `slalom`, `powerline` and `cesium`.

## See Also

- [Quickstart](https://breuerpeter.github.io/nexus/guide/quickstart/) (nexus docs)
- [Running a SITL flight](https://breuerpeter.github.io/nexus/guide/running/) (nexus docs): the full workflow, the ports and the known problems.
- [Simulation](../simulation/index.md): general information about simulators in PX4.
