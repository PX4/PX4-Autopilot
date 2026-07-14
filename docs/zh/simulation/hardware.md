# Hardware Simulation

PX4 can run simulation directly on a real flight controller, replacing real sensors with simulated data, while otherwise executing the full flight stack on actual autopilot hardware.

:::info
Simulating PX4 on flight controller hardware exercises more flight stack code than SITL, and tests more of your hardware integration.
It can surface issues with running PX4 that might hidden when running on a desktop OS and hardware, or even a different flight controller board.
:::

Two simulation approaches are available, controlled by the [SYS_HITL](../advanced_config/parameter_reference.md#SYS_HITL) parameter:

- **[HITL Simulation](../simulation/hitl.md) (`SYS_HITL=1`):** An external simulator (Gazebo Classic or jMAVSim) runs physics on a companion computer and sends sensor data to the flight controller via MAVLink HIL messages. Requires a USB/UART connection and simulator setup.
- **[SIH on Hardware](../sim_sih/hardware.md) (`SYS_HITL=2`):** A C++ physics model runs directly on the flight controller itself. No external simulator, no companion computer, no MAVLink sensor data. Just set the parameter and reboot.

## HITL vs SIH {#comparision}

|                   | HITL (`SYS_HITL=1`)                                 | SIH (`SYS_HITL=2`)                |
| ----------------- | ---------------------------------------------------------------------- | ---------------------------------------------------- |
| Physics model     | External simulator (Gazebo Classic, jMAVSim)        | Internal C++ module                                  |
| Communication     | MAVLink HIL messages                                                   | uORB (internal)                   |
| External process  | Required                                                               | Not required                                         |
| Setup complexity  | Higher                                                                 | Lower                                                |
| Sensor simulation | Camera, lidar, etc. (via simulator) | IMU, GPS, baro, mag, airspeed only                   |
| Vehicle types     | Quadcopter, Standard VTOL                                              | Quad, Hex, FW, VTOL Tailsitter, Standard VTOL, Rover |

## When to Use Which

- Use **SIH** if you want the simplest possible setup. No external dependencies.
- Use **HITL** if you need an external physics engine, 3D visualization from Gazebo Classic, or camera/lidar sensor simulation that SIH does not provide.
