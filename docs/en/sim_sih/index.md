# SIH Simulation

<Badge type="tip" text="PX4 v1.9 (MC)" /><Badge type="tip" text="PX4 v1.13 (MC, VTOL, FW)" />

SIH (Simulation-In-Hardware) is a lightweight, headless simulator with zero external dependencies that runs physics directly inside PX4 via uORB messages.
No GUI, no external processes, no rendering overhead -- just PX4 running a C++ physics model.
This makes it the fastest way to iterate on flight code.

Two modes:

- **SITL (recommended):** Runs on your computer, no hardware needed. Headless by default.
- **On flight controller hardware:** Runs the entire simulation on the autopilot (`SYS_HITL=2`).

SIH addresses key findings from the [PX4 Simulation Survey](https://www.mcguirerobotics.com/px4_sim_research_report/) (K. McGuire, Dronecode Foundation, Dec 2025, 120 respondents): zero-dependency SITL, macOS support, and fast iteration cycles.

## Overview

SIH runs as a PX4 module that replaces real sensor and actuator hardware with a simulated physics model.
It provides simulated IMU, GPS, barometer, magnetometer, and airspeed sensor data via uORB, and reads actuator outputs to update the vehicle state at each timestep.

The simulation runs in lockstep with PX4, ensuring deterministic and reproducible results.

Six vehicle types are supported:

| Vehicle | Make Target |
|---|---|
| Quadrotor X | `sihsim_quadx` |
| Hexarotor X | `sihsim_hexa` (PX4 v1.16+) |
| Fixed-wing (airplane) | `sihsim_airplane` |
| Tailsitter VTOL | `sihsim_xvert` |
| Standard VTOL (QuadPlane) | `sihsim_standard_vtol` (PX4 v1.16+) |
| Ackermann Rover | `sihsim_rover` (PX4 v1.16+) |

### How SIH Works

```
     Plant         Sensors
  Actuator     ┌──────────────┐
  Outputs      │              │     Simulated
  ───────►     │  SIH Module  │     Sensor Data
               │  (C++ uORB)  │     ──────────►
               │              │
               └──────────────┘
```

SIH differs from external simulators:
- **No MAVLink simulator API:** SIH communicates entirely via uORB (PX4's internal message bus).
- **No external process:** The physics model runs in the same PX4 process.
- **Lockstep by default:** Simulation time is synchronized with PX4 scheduling.

### ROS 2 Support

SIH supports ROS 2 via the same mechanism as Gazebo: the [uXRCE-DDS client](../middleware/uxrce_dds.md) that auto-starts in SITL mode.
This exposes 40+ uORB topics to ROS 2 with no additional configuration.
See [ROS 2 Integration](#ros-2-integration) for setup instructions.

## Compatibility

SIH has been tested with PX4 since v1.9 for multirotors and v1.13 for fixed-wing and VTOL.

| Vehicle | PX4 Version |
|---|---|
| Quadrotor | v1.9+ |
| Fixed-wing | v1.13+ |
| Tailsitter VTOL | v1.13+ |
| Hexarotor | v1.16+ |
| Standard VTOL | v1.16+ |
| Ackermann Rover | v1.16+ |

<a id="sih-as-sitl-no-fc"></a>

## SIH as SITL (Recommended)

### Quick Start

```sh
make px4_sitl_sih sihsim_quadx
```

This builds and runs PX4 with SIH physics for a quadrotor. No external simulator, no GUI -- headless by default.

QGroundControl auto-connects on UDP port 14550 -- just open it and you'll see the vehicle.

### Supported Vehicle Types

```sh
# Quadrotor
make px4_sitl_sih sihsim_quadx

# Hexarotor (PX4 v1.16+)
make px4_sitl_sih sihsim_hexa

# Fixed-wing airplane
make px4_sitl_sih sihsim_airplane

# Tailsitter VTOL
make px4_sitl_sih sihsim_xvert

# Standard VTOL (QuadPlane, PX4 v1.16+)
make px4_sitl_sih sihsim_standard_vtol

# Ackermann Rover (PX4 v1.16+)
make px4_sitl_sih sihsim_rover
```

### Lightweight Build Target

The `px4_sitl_sih` build target uses `boards/px4/sitl/sih.px4board`, which avoids pulling in Gazebo dependencies.
If you use the default `px4_sitl` target, all Gazebo libraries will be built even if you only want SIH.

```sh
# Recommended: SIH-specific target (no Gazebo dependencies)
make px4_sitl_sih sihsim_quadx

# NOT recommended for SIH: default target pulls in Gazebo libs
make px4_sitl sihsim_quadx
```

### Change Simulation Speed

SIH supports faster-than-realtime simulation via the `PX4_SIM_SPEED_FACTOR` environment variable:

```sh
# Run at 10x speed
PX4_SIM_SPEED_FACTOR=10 make px4_sitl_sih sihsim_quadx
```

### Wind Simulation

SIH supports setting a wind velocity with the PX4 parameters [`SIH_WIND_N`](../advanced_config/parameter_reference.md#SIH_WIND_E) and [`SIH_WIND_E`](../advanced_config/parameter_reference.md#SIH_WIND_E) [m/s]. The parameters can also be changed during flight to simulate changing wind.

### Set Custom Takeoff Location

The default takeoff location can be set using environment variables:

```sh
export PX4_HOME_LAT=28.452386
export PX4_HOME_LON=-13.867138
export PX4_HOME_ALT=28.5
make px4_sitl_sih sihsim_quadx
```

### ROS 2 Integration

SIH works with ROS 2 via the [uXRCE-DDS](../middleware/uxrce_dds.md) client, which auto-starts in SITL mode.
This is the same mechanism used by Gazebo -- both simulators expose the same set of uORB topics to ROS 2.
The DDS agent connects on UDP port **8888** by default (configurable via `UXRCE_DDS_PRT` parameter or `PX4_UXRCE_DDS_PORT` environment variable).

To use SIH with ROS 2:

1. Start SIH:
   ```sh
   make px4_sitl_sih sihsim_quadx
   ```

2. In a separate terminal, start the Micro XRCE-DDS Agent:
   ```sh
   MicroXRCEAgent udp4 -p 8888
   ```

See [uXRCE-DDS (PX4-ROS 2/DDS Bridge)](../middleware/uxrce_dds.md) for full setup instructions, including agent installation and ROS 2 workspace configuration.

### Port Reference

PX4 SITL opens the following UDP ports (all instance-aware, offset by instance number N).

| PX4 sends to (remote) | PX4 listens on (local) | Use for | Instance offset |
|---|---|---|---|
| **14550** | 18570 (+N) | QGroundControl, GCS tools | Yes |
| **14540** (+N) | 14580 (+N) | MAVSDK, MAVROS, offboard APIs | Yes (capped at 14549 for 10+ instances) |
| **14030** (+N) | 14280 (+N) | Onboard camera/payload | Yes |
| **13280** (+N) | 13030 (+N) | Gimbal control | Yes |
| **19410** (+N) | 19450 (+N) | jMAVSim display-only (SIH only) | Yes |
| **8888** | -- | uXRCE-DDS / ROS 2 | No (use DDS namespace for multi-instance) |

QGC auto-connects on port **14550** by default. MAVSDK connects on **14540**. No manual port configuration needed for single-instance use.

### Multi-Vehicle Simulation

SIH supports multi-vehicle simulation using PX4's instance system.
Each instance gets unique MAVLink ports, a unique system ID, and a separate DDS namespace.

To launch multiple SIH vehicles, first build:

```sh
make px4_sitl_sih sihsim_quadx
```

Then use the multi-instance launch script:

```sh
./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih
```

Or launch instances manually:

```sh
# Terminal 1 (instance 0)
make px4_sitl_sih sihsim_quadx

# Terminal 2 (instance 1)
./build/px4_sitl_sih/bin/px4 -i 1 -d ./build/px4_sitl_sih/etc

# Terminal 3 (instance 2)
./build/px4_sitl_sih/bin/px4 -i 2 -d ./build/px4_sitl_sih/etc
```

Each instance allocates ports automatically (all offset by instance number):

| Instance | MAVLink (18570+N) | MAVLink (14540+N) | DDS (8888) Namespace |
|---|---|---|---|
| 0 | 18570 | 14540 | (default) |
| 1 | 18571 | 14541 | px4_1 |
| 2 | 18572 | 14542 | px4_2 |

See [Port Reference](#port-reference) for the complete list of ports.

### Visualization (Optional) {#sitl-visualization}

SIH is headless by default -- this is a feature, not a limitation.
If you need a visual aid to see what the vehicle is doing, there are two options:

#### QGroundControl

QGC auto-connects on UDP port 14550. Open QGC while SIH is running and the vehicle appears on the map view with attitude, position, and telemetry.

#### jMAVSim (3D Display-Only)

jMAVSim can render a 3D view of the vehicle using MAVLink position data. No physics are simulated in jMAVSim -- it is display-only.

```sh
./Tools/simulation/jmavsim/jmavsim_run.sh -p 19410 -u -q -o
```

Flags: `-a` for airplane model, `-t` for tailsitter model. The `-o` flag enables display-only mode.

See [jMAVSim Display-Only Mode](../sim_jmavsim/index.md#display-only-mode) for details.

<a id="sih-on-flight-controller-hardware"></a>

## SIH on Flight Controller Hardware

SIH can also run on flight controller hardware with `SYS_HITL=2`, replacing real sensors with simulated data while running on the actual autopilot.
See [SIH on Flight Controller Hardware](hardware.md) for setup instructions.

## Adding New Airframes

[Adding a new airframe](../dev_airframes/adding_a_new_frame.md) for use in SIH simulation is much the same as for other use cases.
You still need to configure your vehicle type and [geometry](../config/actuators.md) (`CA_` parameters) and start any other defaults for that specific vehicle.

::: warning
Not every vehicle can be simulated with SIH — there are currently [six supported vehicle types](../advanced_config/parameter_reference.md#SIH_VEHICLE_TYPE) (quadcopter, fixed-wing, tailsitter, standard VTOL, hexacopter, rover), each of which has a relatively rigid implementation in [`sih.cpp`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_sih/sih.cpp).
:::

The specific differences for SIH simulation airframes are listed in the sections below.

### All Variants

- Set all the [Simulation In Hardware](../advanced_config/parameter_reference.md#simulation-in-hardware) parameters (prefixed with `SIH_`) in order to configure the physical model of the vehicle.

  ::: tip
  Make sure that the `SIH_*` parameters and the `CA_*` parameters describe the same vehicle.
  Note that some of these parameters are redundant!
  :::

- `param set-default SYS_HITL 2` to enable SIH on the next boot.

  ::: info
  This also disables input from real sensors.
  For SIH on the FC (only), it also enables the simulated GPS, barometer, magnetometer, and airspeed sensor.

  For SIH on SITL you will need to explicitly enable these sensors as shown below.
  :::

- `param set-default EKF2_GPS_DELAY 0` to improve state estimator performance (the assumption of instant GPS measurements would normally be unrealistic, but is accurate for SIH).

### SIH on Flight Controller

For FC-specific airframe setup (file locations, `HIL_ACT_FUNC*` parameters), see [Adding New Airframes (FC)](hardware.md#adding-new-airframes-fc).

### SIH as SITL

- Airframe file goes in `ROMFS/px4fmu_common/init.d-posix/airframes` and follows the naming template `${ID}_sihsim_${model_name}`, where `ID` is the `SYS_AUTOSTART_ID` used to select the airframe, and `model_name` is the airframe model name.
- Add the model name in `src/modules/simulation/simulator_sih/CMakeLists.txt` to generate a corresponding make target.
- Actuators are configured as usual with `PWM_MAIN_FUNC*` and `PWM_MAIN_REV`.
- Additionally, set:
  - `PX4_SIMULATOR=${PX4_SIMULATOR:=sihsim}`
  - `PX4_SIM_MODEL=${PX4_SIM_MODEL:=svtol}` (replacing `svtol` with the airframe model name)
- Enable the simulated sensors using [SENS_EN_GPSSIM](../advanced_config/parameter_reference.md#SENS_EN_GPSSIM), [SENS_EN_BAROSIM](../advanced_config/parameter_reference.md#SENS_EN_BAROSIM), [SENS_EN_MAGSIM](../advanced_config/parameter_reference.md#SENS_EN_MAGSIM), and [SENS_EN_ARSPDSIM](../advanced_config/parameter_reference.md#SENS_EN_ARSPDSIM) (this is only needed for SIH-as-SITL):
  - `param set-default SENS_EN_GPSSIM 1`
  - `param set-default SENS_EN_BAROSIM 1`
  - `param set-default SENS_EN_MAGSIM 1`
  - `param set-default SENS_EN_ARSPDSIM 1` (if it is a fixed-wing or VTOL airframe with airspeed sensor)

For specific examples see the `_sihsim_` airframes in [ROMFS/px4fmu_common/init.d-posix/airframes](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d-posix/airframes) (SIH as SITL) and [ROMFS/px4fmu_common/init.d/airframes](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d/airframes) (SIH on FC).

## Controlling Actuators on Hardware

See [Controlling Actuators](hardware.md#controlling-actuators) on the SIH on Hardware page.

## Dynamic Models

The dynamic models for the various vehicles are:

- Quadrotor: [pdf](https://github.com/PX4/PX4-Autopilot/raw/main/docs/assets/simulation/SIH_dynamic_model.pdf)
- Fixed-wing: based on Khan (2016), see references below
- Tailsitter: based on Chiappinelli (2018), see references below
- Rover: bicycle model with linear tire model

**References:**

1. PX4 Development Team, "SIH Dynamic Model," PX4-Autopilot, 2019. [PDF](https://github.com/PX4/PX4-Autopilot/raw/main/docs/assets/simulation/SIH_dynamic_model.pdf)
2. W. Khan, "Dynamics modeling of agile fixed-wing unmanned aerial vehicles," Ph.D. thesis, Dept. of Mechanical Engineering, McGill University, Montreal, 2016.
3. R. Chiappinelli, "Modeling and control of a flying wing tailsitter unmanned aerial vehicle," M.Sc. thesis, Dept. of Mechanical Engineering, McGill University, Montreal, 2018.
4. S. Anumakonda, "Everything you need to know about Self-Driving Cars," 2021. [Link](https://srianumakonda.medium.com/everything-you-need-to-know-about-self-driving-in-30-minutes-b38d68bd3427)

## Video

@[youtube](https://youtu.be/PzIpSCRD8Jo)

## Credits

SIH was originally developed by Coriolis g Corporation.
The airplane model and tailsitter models were added by Altitude R&D inc.
Both are Canadian companies:

- Coriolis g developed a new type of Vertical Takeoff and Landing (VTOL) vehicles based on passive coupling systems;
- [Altitude R&D](https://www.altitude-rd.com/) is specialized in dynamics, control, and real-time simulation (today relocated in Zurich).

The simulator is released for free under BSD license.

<!-- original author: @romain-chiap -->
