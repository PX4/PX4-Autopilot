# SIH Simulation

<Badge type="tip" text="PX4 v1.9 (MC)" /><Badge type="tip" text="PX4 v1.13 (MC, VTOL, FW)" /> <Badge type="tip" text="PX4 v1.16 (Rover)" />

SIH (Simulation-In-Hardware) is a lightweight, headless simulator with zero external dependencies that runs physics directly inside PX4 via uORB messages.
No GUI, no external processes, no rendering overhead — just PX4 running a C++ physics model.
This makes it the fastest way to iterate on flight code.

:::tip
SIH is also available as a [prebuilt Docker container or .deb package](../simulation/px4_sitl_prebuilt_packages.md), which is useful if you don't need to modify PX4 itself.
See [PX4 Simulation QuickStart](../simulation/px4_simulation_quickstart.md) for a one-line instruction on how this is used.
:::

## 개요

SIH runs as a PX4 module that replaces real sensor and actuator hardware with a simulated physics model.
It provides simulated IMU, GPS, barometer, magnetometer, and airspeed sensor data via uORB, and reads actuator outputs to update the vehicle state at each timestep.

The simulation runs in lockstep with PX4, ensuring deterministic and reproducible results.
It also integrates seamlessly with ROS 2 with no additional configuration (see [ROS 2 Integration](#ros-2-integration) below).

Two modes are supported:

- **[SITL](#sih-as-sitl-no-fc):** Runs on your computer with no hardware needed, and headless (without a UI) by default.
  _This is the fastest and easiest way to start a simulation on PX4._

- **[SIH on flight controller hardware](#sih-on-flight-controller-hardware):** Runs the entire simulation on the autopilot (`SYS_HITL=2`).

### Supported Vehicle Types

The following vehicle types are supported:

| Vehicle                                                                            | Make Target                              | Status |
| ---------------------------------------------------------------------------------- | ---------------------------------------- | ------ |
| Quadrotor X <Badge type="tip" text="PX4 v1.9" />                                   | `make px4_sitl_sih sihsim_quadx`         | Stable |
| Hexarotor X <Badge type="tip" text="PX4 v1.16" />                                  | `make px4_sitl_sih sihsim_hexa`          | 실험     |
| Fixed-wing (airplane) <Badge type="tip" text="PX4 v1.13" />     | `make px4_sitl_sih sihsim_airplane`      | 실험     |
| Tailsitter VTOL <Badge type="tip" text="PX4 v1.13" />                              | `make px4_sitl_sih sihsim_xvert`         | 실험     |
| Standard VTOL (QuadPlane) <Badge type="tip" text="PX4 v1.16" /> | `make px4_sitl_sih sihsim_standard_vtol` | 실험     |
| Ackermann Rover <Badge type="tip" text="PX4 v1.16" />                              | `make px4_sitl_sih sihsim_rover`         | 실험     |

:::warning
Only the quadrotor vehicle type is stable and recommended for development. All other vehicle types (hexarotor, fixed-wing, VTOL, rover) are experimental and may have aerodynamic model or controller interaction issues that produce unrealistic flight behaviour.
:::

### How SIH Works

![SIH Overview](../../assets/simulation/sih_overview.svg)

SIH differs from external simulators:

- **No MAVLink simulator API:** SIH communicates entirely via uORB (PX4's internal message bus).
- **No external process:** The physics model runs in the same PX4 process.
- **Lockstep by default:** Simulation time is synchronized with PX4 scheduling.

## SIH as SITL {#sih-as-sitl-no-fc}

SIH as SITL is the easiest and fastest way to set up a simulator with PX4.
It requires no hardware, and very few extra dependencies.

### 빠른 시작

To build PX4 and run SIH for a quadrotor:

```sh
make px4_sitl_sih sihsim_quadx
```

QGroundControl auto-connects on UDP port 14550 — just open it and you'll see the vehicle.
Note that the simulation is "headless" by default (has no GUI), but you can use an external viewer.

See [Supported vehicle types](#supported-vehicle-types) for other vehicles.

:::tip
Use the `px4_sitl_sih` build target!
The `px4_sitl` target will work, but will also build Gazebo libraries.
:::

### Visualization (Optional) {#sitl-visualization}

SIH is intentionally headless by default.
If you need a visual aid to see what the vehicle is doing you can use QGroundControl to track path over ground, and/or [Hawkeye](../sim_hawkeye/index.md) as a 3D viewer.

#### QGroundControl

QGC auto-connects on UDP port 14550. Open QGC while SIH is running and the vehicle appears on the map view with attitude, position, and telemetry.

#### Hawkeye (3D Visualizer)

[Hawkeye](../sim_hawkeye/index.md) renders a real-time 3D view of the vehicle using MAVLink position data.
No physics are simulated in Hawkeye — it is a visualizer only.

In a separate terminal, run:

```sh
hawkeye
```

Hawkeye connects on UDP port 19410 by default (the same port SIH sends `HIL_STATE_QUATERNION` on).
It then auto-detects the vehicle type from the MAVLink `HEARTBEAT` and loads the matching 3D model.

The [Hawkeye](../sim_hawkeye/index.md) overview page explains how to install the software.
See the [Hawkeye docs](https://px4.github.io/Hawkeye/) for other features, such as ULog replay, and multi-vehicle visualization.

### Environment Configuration

#### Change Simulation Speed

SIH supports faster-than-realtime simulation via the `PX4_SIM_SPEED_FACTOR` environment variable:

```sh
# Run at 10x speed
PX4_SIM_SPEED_FACTOR=10 make px4_sitl_sih sihsim_quadx
```

#### Wind Simulation

SIH supports setting a wind velocity with the PX4 parameters [`SIH_WIND_N`](../advanced_config/parameter_reference.md#SIH_WIND_E) and [`SIH_WIND_E`](../advanced_config/parameter_reference.md#SIH_WIND_E) [m/s]. The parameters can also be changed during flight to simulate changing wind.

#### Set Custom Takeoff Location

The default takeoff location can be set using environment variables:

```sh
export PX4_HOME_LAT=28.452386
export PX4_HOME_LON=-13.867138
export PX4_HOME_ALT=28.5
make px4_sitl_sih sihsim_quadx
```

### ROS 2 Integration

SIH works with ROS 2 via the [uXRCE-DDS](../middleware/uxrce_dds.md) client, which auto-starts in SITL mode.
This is the same mechanism used by Gazebo — both simulators expose the same set of uORB topics to ROS 2.
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

| PX4 sends to (remote) | PX4 listens on (local) | Use for                                          | Instance offset                                              |
| ---------------------------------------- | ----------------------------------------- | ------------------------------------------------ | ------------------------------------------------------------ |
| **14550**                                | 18570 (+N)             | QGroundControl, GCS tools                        | Yes                                                          |
| **14540** (+N)        | 14580 (+N)             | MAVSDK, MAVROS, offboard APIs                    | Yes (capped at 14549 for 10+ instances)   |
| **14030** (+N)        | 14280 (+N)             | Onboard camera/payload                           | Yes                                                          |
| **13280** (+N)        | 13030 (+N)             | Gimbal control                                   | Yes                                                          |
| **19410** (+N)        | 19450 (+N)             | Hawkeye visualizer (SIH only) | Yes                                                          |
| **8888**                                 | -                                         | uXRCE-DDS / ROS 2                                | No (use DDS namespace for multi-instance) |

QGC auto-connects on port **14550** by default. MAVSDK connects on **14540**. No manual port configuration needed for single-instance use.

### 다중 차량 시뮬레이션

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
| -------- | ------------------------------------ | ------------------------------------ | --------------------------------------- |
| 0        | 18570                                | 14540                                | (default)            |
| 1        | 18571                                | 14541                                | px4_1              |
| 2        | 18572                                | 14542                                | px4_2              |

See [Port Reference](#port-reference) for the complete list of ports.

## Running the SIH on Flight Controller Hardware {#sih-on-flight-controller-hardware}

:::info
The SIH on flight controller is community supported.
:::

SIH can also run on flight controller hardware, replacing real sensors with simulated data while running on the actual autopilot.
See [SIH on Flight Controller Hardware](hardware.md) for setup instructions.

## Adding New Airframes

[Adding a new airframe](../dev_airframes/adding_a_new_frame.md) for use in SIH simulation is much the same as for other use cases.
You still need to configure your vehicle type and [geometry](../config/actuators.md) (`CA_` parameters) and start any other defaults for that specific vehicle.

:::warning
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

- `param set-default SENS_GPS0_DELAY 0` to improve state estimator performance (the assumption of instant GPS measurements would normally be unrealistic, but is accurate for SIH).

### SIH on Flight Controller

See [Adding New Airframes (FC)](../sim_sih/hardware.md#adding-new-airframes-fc) in _SIH on Flight Controller Hardware_.

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

## Dynamic Models

The dynamic models for the various vehicles are:

- Quadrotor: [pdf](https://github.com/PX4/PX4-Autopilot/raw/main/docs/assets/simulation/SIH_dynamic_model.pdf)
- Fixed-wing: based on Khan (2016), see references below
- Tailsitter: based on Chiappinelli (2018), see references below
- Rover: bicycle model with linear tire model

**Propeller model with advance ratio**

Since PX4 v1.17, the propeller model for fixed-wing, tailsitter, and VTOL pusher vehicles is based on the equations from UIUC Propeller Database.

<img width="588" height="183" alt="UIUC_prop_equations" src="https://github.com/user-attachments/assets/55413486-b23b-4269-9ac5-dd630ec0849b" />

This model includes the thrust coefficient CT(J) and power coefficient CP(J) as functions of the advance ratio J.
As a result, the maximum thrust force is realistically reduced as the aircraft speed is increased.
The SIH implements the thrust and power coefficients as second-order polynomial fits.

CT = SIH_F_CT0 + SIH_F_CT1⋅J + SIH_F_CT2⋅J²

CP = SIH_F_CP0 + SIH_F_CP1⋅J + SIH_F_CP2⋅J²

If `SIH_F_CT0` and `SIH_F_CP0` are non-zero and positive, the SIH uses the model with advance ratio.
If not, the SIH uses a simple model with maximum thrust force given by `SIH_F_T_MAX` and maximum torque given by `SIH_F_Q_MAX`.

**References:**

1. PX4 Development Team, "SIH Dynamic Model," PX4-Autopilot, 2019. [PDF](https://github.com/PX4/PX4-Autopilot/raw/main/docs/assets/simulation/SIH_dynamic_model.pdf)
2. W. Khan, "Dynamics modeling of agile fixed-wing unmanned aerial vehicles," Ph.D. thesis, Dept. of Mechanical Engineering, McGill University, Montreal, 2016.
3. R. Chiappinelli, "Modeling and control of a flying wing tailsitter unmanned aerial vehicle," M.Sc. thesis, Dept. of Mechanical Engineering, McGill University, Montreal, 2018.
4. S. Anumakonda, "Everything you need to know about Self-Driving Cars," 2021. [Link](https://srianumakonda.medium.com/everything-you-need-to-know-about-self-driving-in-30-minutes-b38d68bd3427)
5. J.B. Brandt, R.W. Deters, G.K. Ananda, O.D. Dantsker, and M.S. Selig, UIUC Propeller Database, Vols 1-4, University of Illinois at Urbana-Champaign, Department of Aerospace Engineering, retrieved from https://m-selig.ae.illinois.edu/props/propDB.html.

## 비디오

SIH demo with a fixed-wing vehicle @[youtube](https://youtu.be/PzIpSCRD8Jo)  
How to parametrize the thrust and power coefficients CT & CP @[youtube](https://www.youtube.com/watch?v=KNSd9ge0sSw)

## Credits

SIH was originally developed by Coriolis g Corporation.
The airplane model and tailsitter models were added by Altitude R&D inc.

- Coriolis g developed a new type of Vertical Takeoff and Landing (VTOL) vehicles based on passive coupling systems;
- [Altitude R&D](https://www.altitude-rd.com/) is specialized in dynamics, control, and real-time simulation (located in Zurich).

The simulator is released for free under BSD license.

<!-- original author: @romain-chiap -->
