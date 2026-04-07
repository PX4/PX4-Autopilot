# Try PX4 Simulation

PX4 runs a full autopilot in simulation, on your laptop, with nothing to install besides Docker.
One command and you have a flying vehicle ready for QGroundControl, MAVSDK, or ROS 2.

```sh
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl-sih:latest
```

That's it.
PX4 is running.
Open [QGroundControl](https://qgroundcontrol.com) and fly.

No hardware, no build environment, no dependencies beyond [Docker](https://docs.docker.com/get-docker/).
Read on for more options.

## What This Runs

The command above starts a **lightweight simulation** where the physics run inside PX4 itself.
It starts instantly and works everywhere, but it is headless (no 3D window).
You can still monitor the vehicle in QGroundControl, and there are [additional visualization options](../sim_sih/index.md#sitl-visualization) if you need them.

If you need a **3D environment** with cameras, LiDAR, and custom worlds, PX4 also supports [Gazebo](../sim_gazebo_gz/index.md).
Gazebo requires more setup (graphics forwarding, GPU access), so it is better suited as a [native install](../simulation/px4_sitl.md) or [built from source](building_px4.md).
See the [simulator comparison table](../simulation/index.md#simulator-comparison) for a full feature breakdown.

## Container (any OS)

Containers package PX4 and all its dependencies into a single download so you don't have to install anything on your system.
Just install [Docker](https://docs.docker.com/get-docker/) (a free tool that runs containers) and you're ready to go.

This command then works on Linux, macOS, and Windows:

```sh
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl-sih:latest
```

PX4 starts a simulated quadcopter.
You land in the PX4 shell (`pxh>`).
Open [QGroundControl](https://qgroundcontrol.com) and it auto-connects on UDP port 14550.

For other vehicle types, environment variables, and configuration, see the full [SIH Simulation](../sim_sih/index.md) guide.

## Native Install (Ubuntu)

If you're on Ubuntu and prefer to install PX4 simulation (SITL) directly on your system instead of using a container, you can download a `.deb` package.
This installs PX4 SITL as a regular application — no Docker required.
Both lightweight and Gazebo 3D packages are available.

Available for Ubuntu 22.04 and 24.04.
Download from the [PX4 Releases](https://github.com/PX4/PX4-Autopilot/releases) page, then:

```sh
sudo apt install ./px4_*.deb
PX4_SIM_MODEL=sihsim_quadx px4
```

For Gazebo packages, multi-instance, ROS 2 integration, and all configuration options, see the [Pre-built SITL Packages](../simulation/px4_sitl.md) guide.

## Now What?

PX4 is running.
Here's what to try next:

1. **Fly around.**
   Open [QGroundControl](https://qgroundcontrol.com) — it connects automatically.
   Arm the vehicle, take off, and explore the flight modes.
2. **Control it with code.**
   Use [MAVSDK](https://mavsdk.mavlink.io/) to write a program that commands the vehicle: takeoff, fly a mission, land.
   This supports many programming languages (it uses the MAVLink protocol for the underlying connection).
3. **Connect ROS 2.**
   PX4 publishes vehicle state and accepts commands over [uXRCE-DDS](../ros2/user_guide.md).
   If ROS 2 is your stack, you can subscribe to topics and send setpoints from ROS nodes.
4. **Go deeper.**
   Ready to modify PX4 itself? Head to the next page to set up a development environment and build from source.
