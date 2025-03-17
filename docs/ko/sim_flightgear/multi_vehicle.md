# Multi-Vehicle Simulation with FlightGear

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

This topic explains how to simulate multiple vehicles using FlightGear in SITL.
All vehicle instances have parameters defined by their startup scripts.

:::info
This is the most environmentally realistic way to simulate multiple vehicles running PX4, and allows easy testing of multiple different types of vehicles.
It is suitable for testing multi-vehicle support in _QGroundControl_, [MAVSDK](https://mavsdk.mavlink.io/), etc.

[Multi-Vehicle Simulation with Gazebo Classic](../sim_gazebo_classic/multi_vehicle_simulation.md) should be used instead for: swarm simulations with many vehicles, and testing features like computer vision that are only supported by Gazebo Classic.
:::

## How to Start Multiple Instances

To start multiple instances (on separate ports and IDs):

1. Checkout the [PX4 branch that supports multiple vehicles](https://github.com/ThunderFly-aerospace/PX4Firmware/tree/flightgear-multi) (at ThunderFly-aerospace):

   ```sh
   git clone https://github.com/ThunderFly-aerospace/PX4Firmware.git
   cd PX4Firmware
   git checkout flightgear-multi
   ```

2. Build the PX4 Firmware using the standard toolchain (with FlightGear installed).

3. Start the first instance using the [predefined scripts](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge/tree/master/scripts):

   ```sh
   cd ./Tools/flightgear_bridge/scripts
   ./vehicle1.sh
   ```

4. Start subsequent instances using another script:

   ```sh
   ./vehicle2.sh
   ```

Each instance should have its own startup script, which can represent a completely different vehicle type.
For prepared scripts you should get the following view.

![Multi-vehicle simulation using PX4 SITL and FlightGear](../../assets/simulation/flightgear/flightgear-multi-vehicle-sitl.jpg)

Ground stations such as _QGroundControl_ connect to all instances using the normal UDP port 14550 (all traffic goes to the same port).

The number of simultaneously running instances is limited mainly by computer resources.
FlightGear is a single-thread application, but aerodynamics solvers consume a lot of memory.
Therefore splitting to multiple computers and using a [multiplayer server](https://wiki.flightgear.org/Howto:Multiplayer) is probably required to run _many_ vehicle instances.

## Additional Resources

- See [Simulation](../simulation/index.md) for more information about the port configuration.
