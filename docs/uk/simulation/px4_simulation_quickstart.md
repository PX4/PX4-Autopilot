# PX4 Simulation QuickStart

First install [Docker](https://docs.docker.com/get-docker/) (a free tool that runs containers).

The following command will then run a PX4 quadrotor simulation that you can connect to [QGroundControl](https://qgroundcontrol.com), [MAVSDK](https://mavsdk.mavlink.io/) or [ROS 2](../ros2/user_guide.md) (on Linux, macOS, and Windows):

```sh
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl:latest
```

That's it — open [QGroundControl](https://qgroundcontrol.com) and fly!

::: tip

To try [other vehicle types](../sim_sih/#supported-vehicle-types) append the corresponding line below to the command:

```sh
-e PX4_SIM_MODEL=sihsim_airplane # Plane
-e PX4_SIM_MODEL=sihsim_standard_vtol # Standard VTOL
-e PX4_SIM_MODEL=sihsim_rover # Ackermann rover
```

For more information and options see [Container Images](../simulation/px4_sitl_prebuilt_packages.md#container-images) (in _Pre-built SITL Packages_) and [SIH Simulation](../sim_sih/index.md).

:::
