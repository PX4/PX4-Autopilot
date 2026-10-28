# PX4 Simulation QuickStart

First install [Docker](https://docs.docker.com/get-started/get-docker/) (a free tool that runs containers).

The following command will then run a PX4 quadrotor simulation that you can connect to [QGroundControl](https://qgroundcontrol.com), [MAVSDK](https://mavsdk.mavlink.io/) or [ROS 2](../ros2/user_guide.md).

```sh
docker run --rm -it px4io/px4-sitl:latest
```

On Linux (or WSL2 without _Docker Desktop_):

```sh
docker run --rm -it --network host px4io/px4-sitl:latest
```

That's it — open [QGroundControl](https://qgroundcontrol.com) and fly!

::: tip

To try [other vehicle types](../sim_sih/#supported-vehicle-types), use the `-e` flag to pass the `PX4_SIM_MODEL` environment variable to the `docker run` command (on Linux, or WSL2 without _Docker Desktop_, also add `--network host`):

飞机

```sh
docker run --rm -it -e PX4_SIM_MODEL=sihsim_airplane px4io/px4-sitl:latest
```

标准垂起固定翼

```sh
docker run --rm -it -e PX4_SIM_MODEL=sihsim_standard_vtol px4io/px4-sitl:latest
```

Ackermann rover

```sh
docker run --rm -it -e PX4_SIM_MODEL=sihsim_rover_ackermann px4io/px4-sitl:latest
```

For more information and options see [Container Images](../simulation/px4_sitl_prebuilt_packages.md#container-images) (in _Pre-built SITL Packages_) and [SIH Simulation](../sim_sih/index.md).

:::
