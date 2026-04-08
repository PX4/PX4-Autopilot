# PX4 Simulation QuickStart

First install [Docker](https://docs.docker.com/get-docker/).
The following command will then run a PX4 quadrotor simulation that you can connect to [QGroundControl](https://qgroundcontrol.com), [MAVSDK](https://mavsdk.mavlink.io/) or [ROS 2](../ros2/user_guide.md) (on Linux, macOS, and Windows):

```sh
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl:latest
```

That's it — open [QGroundControl](https://qgroundcontrol.com) and fly.

For other vehicle types, environment variables, and configuration, see the full [SIH Simulation](../sim_sih/index.md) guide.

## What This Runs

The command above immediately starts a **lightweight simulation** of PX4 for a quadcopter, running in a container in _Docker_.
Containers package PX4 and all its dependencies into a single download so you don't have to install anything on your system.
Just install [Docker](https://docs.docker.com/get-docker/) (a free tool that runs containers) and you're ready to go.

The container runs the [SIH simulator](../sim_sih/index), which executes the physics of the simulation within PX4 itself.
The simulation is headless by default (no 3D window), but you can still monitor the vehicle in QGroundControl.
There are also [additional visualization options](../sim_sih/index.md#sitl-visualization) if you need them.

Once PX4 has started the docker window will open and run the PX4 shell (`pxh>`).
When you open [QGroundControl](https://qgroundcontrol.com), it auto-connects on UDP port 14550.

If you need a **3D environment** with cameras, LiDAR, and custom worlds, PX4 also supports [Gazebo](../sim_gazebo_gz/index.md).
Gazebo requires more setup (graphics forwarding, GPU access), so it is better to run it as a [native install](../simulation/px4_sitl_prebuilt_packages.md) or [built from source](building_px4.md).
See the [simulator comparison table](../simulation/index.md#simulator-comparison) for a full feature breakdown.

## Native Install (Ubuntu)

On Ubuntu (only) you can alternatively install the PX4 simulation as a native application (instead of using a container running in Docker).
Both lightweight and Gazebo 3D packages are available for Ubuntu 22.04 and 24.04.

Download the desired `.deb` files from the [PX4 Releases](https://github.com/PX4/PX4-Autopilot/releases) page, then install them as shown:

```sh
sudo apt install ./px4_*.deb
PX4_SIM_MODEL=sihsim_quadx px4
```

For Gazebo packages, multi-instance, ROS 2 integration, and all configuration options, see the [Pre-built SITL Packages](../simulation/px4_sitl_prebuilt_packages.md) guide.

## Now What?

Once PX4 is running, you can try out the following options:

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
