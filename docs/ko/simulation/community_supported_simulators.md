# Community Supported Simulators

This section contains information about _community-supported_ simulations

:::warning
These simulators are not maintained, tested, or supported, by the core development team.
They may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

The tools have variable levels of support from their communities (some are well supported and others are not).
Questions about these tools should be raised on the [discussion forums](../contribute/support.md#forums-and-chat)

| 시뮬레이터                                                                  | 설명                                                                                                                                                                                                                                                                                                                                                                                                                         |
| ---------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Simulation-In-Hardware](../sim_sih/index.md) (SIH) | <p>A simulator implemented in C++ as a PX4 module directly in the Firmware [code](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/simulator_sih). It can be ran in SITL directly on the computer or as an alternative to HITL offering a hard real-time simulation directly on the hardware autopilot.  </p><p><strong>Supported Vehicles:</strong> Quad, Hexa, Plane, Tailsitter, Standard VTOL</p> |
| [FlightGear](../sim_flightgear/index.md)                               | <p>A simulator that provides physically and visually realistic simulations. In particular it can simulate many weather conditions, including thunderstorms, snow, rain and hail, and can also simulate thermals and different types of atmospheric flows. [Multi-vehicle simulation](../sim_flightgear/multi_vehicle.md) is also supported.</p> <p><strong>Supported Vehicles:</strong> Plane, Autogyro, Rover</p>         |
| [JMAVSim](../sim_jmavsim/index.md)                                     | <p>A simple multirotor/quad simulator. This was previously part of the PX4 development toolchain but was removed in favour of [Gazebo](../sim_gazebo_gz/index.md).</p> <p><strong>Supported Vehicles:</strong> Quad</p>                                                                                                                                                                                                    |
| [JSBSim](../sim_jsbsim/index.md)                                       | <p>A simulator that provides advanced flight dynamics models. This can be used to model realistic flight dynamics based on wind tunnel data.</p> <p><strong>Supported Vehicles:</strong> Plane, Quad, Hex</p>                                                                                                                                                                                                              |
| [AirSim](../sim_airsim/index.md)                                       | <p>A cross platform simulator that provides physically and visually realistic simulations. This simulator is resource intensive, and requires a significantly more powerful computer than the other simulators described here.</p><p><strong>Supported Vehicles:</strong> Iris (MultiRotor model and a configuration for PX4 QuadRotor in the X configuration).</p>                                                        |
