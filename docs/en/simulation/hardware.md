# Hardware Simulation

PX4 can run simulation directly on a real flight controller, replacing real sensors with simulated data while executing the full flight stack on actual autopilot hardware.
This is useful for testing without propellers, validating firmware on specific boards, and verifying hardware integration without risk.

Two approaches are available, controlled by the `SYS_HITL` parameter:

- **HITL (`SYS_HITL=1`):** An external simulator (Gazebo Classic or jMAVSim) runs physics on a companion computer and sends sensor data to the flight controller via MAVLink HIL messages. Requires a USB/UART connection and simulator setup.
- **SIH (`SYS_HITL=2`):** A C++ physics model runs directly on the flight controller itself. No external simulator, no companion computer, no MAVLink sensor data. Just set the parameter and reboot.

## Comparison

| | HITL (`SYS_HITL=1`) | SIH (`SYS_HITL=2`) |
|---|---|---|
| Physics model | External simulator (Gazebo Classic, jMAVSim) | Internal C++ module |
| Communication | MAVLink HIL messages | uORB (internal) |
| External process | Required | Not required |
| Setup complexity | Higher | Lower |
| Sensor simulation | Camera, lidar, etc. (via simulator) | IMU, GPS, baro, mag, airspeed only |
| Vehicle types | Quadcopter, Standard VTOL | Quad, Hex, FW, Tailsitter, Standard VTOL, Rover |

## When to Use Which

- Use **SIH** if you want the simplest possible setup for testing on hardware without propellers. No external dependencies.
- Use **HITL** if you need an external physics engine, 3D visualization from Gazebo Classic, or camera/lidar sensor simulation that SIH does not provide.

## Setup

- [HITL Simulation](hitl.md): Traditional hardware-in-the-loop with an external simulator.
- [SIH on Hardware](../sim_sih/hardware.md): Simulation-In-Hardware running entirely on the flight controller.
