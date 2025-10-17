# Simulation-In-Hardware (SIH)

<Badge type="tip" text="PX4 v1.9 (MC)" /><Badge type="tip" text="PX4 v1.13 (MC, VTOL, FW)" />

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4 (known to work in PX4 v1.14).

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

Simulation-In-Hardware (SIH) is an alternative to [Hardware In The Loop simulation (HITL)](../simulation/hitl.md) for quadrotors, fixed-wing vehicles (airplane), and VTOL tailsitters.

SIH can be used by new PX4 users to get familiar with PX4 and the different modes and features, and of course to learn to fly a vehicle using an RC controller in simulation, which is not possible using SITL.

## 综述

With SIH the whole simulation is running on embedded hardware: the controller, the state estimator, and the simulator.
The Desktop computer is only used to display the virtual vehicle.

![Simulator MAVLink API](../../assets/diagrams/SIH_diagram.png)

### Compatibility

- SIH is compatible with all PX4 supported boards except those based on FMUv2.
- SIH for MC quadrotor is supported from PX4 v1.9.
- SIH for FW (airplane) and VTOL tailsitter are supported from PX4 v1.13.
- SIH as SITL (without hardware) from PX4 v1.14.
- SIH for Standard VTOL from PX4 v1.16.
- SIH for MC Hexacopter X from `main` (expected to be PX4 v1.17).
- SIH for Ackermann Rover from `main`.

### Benefits

SIH provides several benefits over HITL:

- It ensures synchronous timing by avoiding the bidirectional connection to the computer.
  As a result the user does not need such a powerful desktop computer.
- The whole simulation remains inside the PX4 environment.
  Developers who are familiar with PX4 can more easily incorporate their own mathematical model into the simulator.
  They can, for instance, modify the aerodynamic model, or noise level of the sensors, or even add a sensor to be simulated.
- The physical parameters representing the vehicle (such as mass, inertia, and maximum thrust force) can easily be modified from the [SIH parameters](../advanced_config/parameter_reference.md#simulation-in-hardware).

## Requirements

To run the SIH, you will need a:

- [Flight controller](../flight_controller/index.md), such as a Pixhawk-series board.

  ::: info
  From PX4 v1.14 you can run [SIH "as SITL"](#sih-as-sitl-no-fc), in which case a flight controller is not required.

:::

- [Manual controller](../getting_started/px4_basic_concepts.md#manual-control): either a [radio control system](../getting_started/rc_transmitter_receiver.md) or a [joystick](../config/joystick.md).

- QGroundControl for flying the vehicle via GCS.

- Development computer for visualizing the virtual vehicle (optional).

## Check if SIH is in Firmware

The modules required for SIH are built into most PX4 firmware by default.
These include: [`pwm_out_sim`](../modules/modules_driver.md#pwm-out-sim), [`sensor_baro_sim`](../modules/modules_system.md#sensor-baro-sim), [`sensor_gps_sim`](../modules/modules_system.md#sensor-gps-sim) and [`sensor_mag_sim`](../modules/modules_system.md#sensor-mag-sim).

To check that these are present on your flight controller:

1. 启动QGroundControl。

2. Open **Analyze Tools > Mavlink Console**.

3. Enter the following commands in the console:

   ```sh
   pwm_out_sim status
   ```

   ```sh
   sensor_baro_sim status
   ```

   ```sh
   sensor_gps_sim status
   ```

   ```sh
   sensor_mag_sim status
   ```

   ::: tip
   Note that when using SIH on real hardware you do not need to additionally enable the modules using their corresponding parameters ([SENS_EN_GPSSIM](../advanced_config/parameter_reference.md#SENS_EN_GPSSIM), [SENS_EN_BAROSIM](../advanced_config/parameter_reference.md#SENS_EN_BAROSIM), [SENS_EN_MAGSIM](../advanced_config/parameter_reference.md#SENS_EN_MAGSIM)).

:::

4. If a valid status is returned you can start using SIH.

If any of the returned values above are `nsh: MODULENAME: command not found`, then you don't have the module installed.
In this case you will have to add them to your board configuration and then rebuild and install the firmware.

### Adding SIH to the Firmware

Add the following key to the configuration file for your flight controller to include all the required modules (for an example see [boards/px4/fmu-v6x/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6x/default.px4board)).
Then re-build the firmware and flash it to the board.

```text
CONFIG_MODULES_SIMULATION_SIMULATOR_SIH=y
```

:::details
What does this do?

This installs the dependencies in [simulator_sih/Kconfig](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_sih/Kconfig).
It is equivalent to:

```text
CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
CONFIG_MODULES_SIMULATION_SENSOR_BARO_SIM=y
CONFIG_MODULES_SIMULATION_SENSOR_GPS_SIM=y
CONFIG_MODULES_SIMULATION_SENSOR_MAG_SIM=y
```

:::

As an alterative to updating configuration files manually, you can use the following command to launch a GUI configuration tool, and interactively enable the required modules at the path: **modules > Simulation > simulator_sih**.
For example, to update the fmu-v6x configuration you would use:

```sh
make px4_fmu-v6x boardconfig
```

After uploading, check that the required modules are present.

:::note
To use rover in SIH you must use the [rover build](../config_rover/index.md#flashing-the-rover-build) or add the rover modules to your board configuration.
:::

## Starting SIH

To set up/start SIH:

1. Connect the flight controller to the desktop computer with a USB cable.
2. Open QGroundControl and wait for the flight controller too boot and connect.
3. Open [Vehicle Setup > Airframe](../config/airframe.md) then select the desired frame:
   - [SIH Quadcopter X](../airframes/airframe_reference.md#copter_simulation_sih_quadcopter_x)
   - **SIH Hexacopter X** (currently only has an airframe for SITL to safe flash so on flight control hardware it has to be manually configured equivalently).
   - [SIH plane AERT](../airframes/airframe_reference.md#plane_simulation_sih_plane_aert)
   - [SIH Tailsitter Duo](../airframes/airframe_reference.md#vtol_simulation_sih_tailsitter_duo)
   - [SIH Standard VTOL QuadPlane](../airframes/airframe_reference.md#vtol_simulation_sih_standard_vtol_quadplane)
   - [SIH Ackermann Rover](../airframes/airframe_reference.md#rover_rover_sih_rover_ackermann)

The autopilot will then reboot.
The `sih` module is started on reboot, and the vehicle should be displayed on the ground control station map.

:::warning
The airplane needs to takeoff in manual mode at full throttle.
Also, if the airplane crashes the state estimator might lose its fix.
:::

## Display/Visualisation (optional)

The SIH-simulated vehicle can be displayed using [jMAVSim](../sim_jmavsim/index.md) as a visualiser.

:::tip
SIH does not _need_ a visualiser — you can connect with QGroundControl and fly the vehicle without one.
:::

To display the simulated vehicle:

1. Close _QGroundControl_ (if open).

2. Unplug and replug the flight controller (allow a few seconds for it to boot).

3. Start jMAVSim by calling the script **jmavsim_run.sh** from a terminal:

   ```sh
   ./Tools/simulation/jmavsim/jmavsim_run.sh -q -d /dev/ttyACM0 -b 2000000 -o
   ```

   where the flags are:

   - `-q` to allow the communication to _QGroundControl_ (optional).
   - `-d` to start the serial device `/dev/ttyACM0` on Linux.
     On macOS this would be `/dev/tty.usbmodem1`.
   - `-b` to set the serial baud rate to `2000000`.
   - `-o` to start jMAVSim in _display Only_ mode (i.e. the physical engine is turned off and jMAVSim only displays the trajectory given by the SIH in real-time).
   - add a flag `-a` to display an aircraft or `-t` to display a tailsitter.
     If this flag is not present a quadrotor will be displayed by default.

4. After few seconds, _QGroundControl_ can be opened again.

At this point, the system can be armed and flown.
The vehicle can be observed moving in jMAVSim, and on the QGC _Fly_ view.

## SIH as SITL (no FC)

SIH can be run as SITL (Software-In-The-Loop) from v1.14.
What this means is that the simulation code is executed on the laptop/computer instead of a flight controller, similar to Gazebo or jMAVSim.
In this case you don't need the flight controller hardware.

To run SIH as SITL:

1. Install the [PX4 Development toolchain](../dev_setup/dev_env.md).
2. Run the appropriate make command for each vehicle type (at the root of the PX4-Autopilot repository):
   - Quadcopter

     ```sh
     make px4_sitl sihsim_quadx
     ```

   - Hexacopter

     ```sh
     make px4_sitl sihsim_hex
     ```

   - Fixed-wing (plane)

     ```sh
     make px4_sitl sihsim_airplane
     ```

   - XVert VTOL tailsitter

     ```sh
     make px4_sitl sihsim_xvert
     ```

   - 标准垂起固定翼

     ```sh
     make px4_sitl sihsim_standard_vtol
     ```

   - Ackermann Rover

     ```sh
     make px4_sitl sihsim_rover_ackermann
     ```

### Change Simulation Speed

SITL allows the simulation to be run faster than real time.
To run the airplane simulation 10 times faster than real time, run the command:

```sh
PX4_SIM_SPEED_FACTOR=10 make px4_sitl sihsim_airplane
```

To display the vehicle in jMAVSim during SITL mode, enter the following command in another terminal:

```sh
./Tools/simulation/jmavsim/jmavsim_run.sh -p 19410 -u -q -o
```

- add a flag `-a` to display an aircraft or `-t` to display a tailsitter.
  If this flag is not present a quadrotor will be displayed by default.

### Set Custom Takeoff Location

The takeoff location in SIH on SITL can be set using environment variables.
This will override the default takeoff location.

The variables to set are: `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`.

例如：

```sh
export PX4_HOME_LAT=28.452386
export PX4_HOME_LON=-13.867138
export PX4_HOME_ALT=28.5
make px4_sitl sihsim_quadx
```

## Adding New Airframes

[Adding a new airframe](../dev_airframes/adding_a_new_frame.md) for use in SIH simulation is much the same as for other use cases.
You still need to configure your vehicle type and [geometry](../config/actuators.md) (`CA_` parameters) and start any other defaults for that specific vehicle.

:::warning
Not every vehicle can be simulated with SIH — there are currently [four supported vehicle types](../advanced_config/parameter_reference.md#SIH_VEHICLE_TYPE), each of which has a relatively rigid implementation in [`sih.cpp`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_sih/sih.cpp).
:::

The specific differences for SIH simulation airframes are listed in the sections below.

For all variants of SIH:

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

For SIH on FC:

- Airframe file goes in `ROMFS/px4fmu_common/init.d/airframes` and follows the naming template `${ID}_${model_name}.hil`, where `ID` is the `SYS_AUTOSTART_ID` used to select the airframe, and `model_name` is the airframe model name.
- Add the model name in `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` to generate a corresponding make target.
- Actuators are configured with `HIL_ACT_FUNC*` parameters (not the usual `PWM_MAIN_FUNC*` parameters).
  This is to avoid using the real actuator outputs in SIH.
  Similarly, the bitfield for inverting individual actuator output ranges is `HIL_ACT_REV`, rather than `PWM_MAIN_REV`.

For SIH as SITL (no FC):

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

For specific examples see the `_sihsim_` airframes in [ROMFS/px4fmu_common/init.d-posix/airframes](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/airframes/) (SIH as SITL) and [ROMFS/px4fmu_common/init.d/airframes](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d/airframes) (SIH on FC).

## Controlling Actuators in SIH

:::warning
If you want to control throttling actuators in SIH, make sure to remove propellers for safety.
:::

In some scenarios, it may be useful to control an actuator while running SIH. For example, you might want to verify that winches or grippers are functioning correctly by checking the servo responses.

To enable actuator control in SIH:

1. Configure PWM parameters in the airframe file:

Ensure your airframe file includes the necessary parameters to map PWM outputs to the correct channels.

For example, if a servo is connected to MAIN 3 and you want to map it to AUX1 on your RC, use the following command:

`param set-default PWM_MAIN_FUNC3 407`

You can find a full list of available values for `PWM_MAIN_FUNCn` [here](../advanced_config/parameter_reference.md#PWM_MAIN_FUNC1). In this case, `407` maps the MAIN 3 output to AUX1 on the RC.

Alternatively, you can use the [`PWM_AUX_FUNCn`](../advanced_config/parameter_reference.md#PWM_AUX_FUNC1) parameters.

You may also configure the output as desired:

- Disarmed PWM: ([`PWM_MAIN_DISn`](../advanced_config/parameter_reference.md#PWM_MAIN_DIS1) / [`PWM_AUX_DIS1`](../advanced_config/parameter_reference.md#PWM_AUX_DIS1))
- Minimum PWM ([`PWM_MAIN_MINn`](../advanced_config/parameter_reference.md#PWM_MAIN_MIN1) / [`PWM_AUX_MINn`](../advanced_config/parameter_reference.md#PWM_AUX_MIN1))
- Maximum PWM ([`PWM_MAIN_MAXn`](../advanced_config/parameter_reference.md#PWM_MAIN_MAX1) / [`PWM_AUX_MAXn`](../advanced_config/parameter_reference.md#PWM_AUX_MAX1))

2. Manually start the PWM output driver

For safety, the PWM driver is not started automatically in SIH. To enable it, run the following command in the MAVLink shell:

`pwm_out start`

And to disable it again:

`pwm_out stop`

## Dynamic Models

The dynamic models for the various vehicles are:

- Quadcopter: [pdf report](https://github.com/PX4/PX4-Autopilot/raw/main/docs/assets/simulation/SIH_dynamic_model.pdf).
- Hexacopter: Equivalent to the Quadcopter but with a symmetric hexacopter x actuation setup.
- Fixed-wing: Inspired by the PhD thesis: "Dynamics modeling of agile fixed-wing unmanned aerial vehicles." Khan, Waqas, supervised by Nahon, Meyer, McGill University, PhD thesis, 2016.
- Tailsitter: Inspired by the master's thesis: "Modeling and control of a flying wing tailsitter unmanned aerial vehicle." Chiappinelli, Romain, supervised by Nahon, Meyer, McGill University, Masters thesis, 2018.
- Ackermann Rover: Based on lateral vehicle dynamics of the bicycle model adapted from [Sri Anumakonda, Everything you need to know about Self-Driving Cars in <30 minutes](https://srianumakonda.medium.com/everything-you-need-to-know-about-self-driving-in-30-minutes-b38d68bd3427)

## 视频

<lite-youtube videoid="PzIpSCRD8Jo" title="SIH FW demo"/>

## Credits

SIH was originally developed by Coriolis g Corporation.
The airplane model and tailsitter models were added by Altitude R&D inc.
Both are Canadian companies:

- Coriolis g developed a new type of Vertical Takeoff and Landing (VTOL) vehicles based on passive coupling systems;
- [Altitude R&D](https://www.altitude-rd.com/) is specialized in dynamics, control, and real-time simulation (today relocated in Zurich).

The simulator is released for free under BSD license.

<!-- original author: @romain-chiap -->
