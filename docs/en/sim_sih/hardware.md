# SIH on Flight Controller Hardware

[SIH](../sim_sih/index.md) can run directly on flight controller hardware with `SYS_AUTOSTART` set to the desired value for the frame.
This replaces real sensors with simulated data while running on the actual autopilot.

For a comparison of SIH and HITL on hardware, see [Hardware Simulation](../simulation/hardware.md).

## Starting SIH

1. Connect the flight controller to QGroundControl via USB.
2. Set `SYS_AUTOSTART` parameter to the desired airframe.
3. Reboot the flight controller.
4. The SIH module starts automatically and provides simulated sensor data.

::: tip
To ensure there is no leftover parameter from previous setup, it is recommended to reset all the parameters to firmware's default before modifying `SYS_AUTOSTART`.
:::

The following airframes are supported.

| SIH Airframe    | SYS_AUTOSTART | Status       |
| --------------- | ------------- | ------------ |
| Quadrotor X     | 1100          | Stable       |
| Airplane        | 1101          | Experimental |
| Tailsitter Duo  | 1102          | Experimental |
| Standard VTOL   | 1103          | Experimental |
| Ackermann Rover | 1104          | Experimental |
| Hexacopter X    | 1105          | Experimental |

Once running, the vehicle can be controlled from QGroundControl or an RC controller.

## Firmware Builds with SIH

The SIH module is included in many, but not all, default firmware builds.
This list can change between PX4 releases.
Always verify using the method in [Check if SIH is in Firmware](#check-if-sih-is-in-firmware).

The table below lists build targets that include SIH at the time of writing:

| Build Target                         | Board                      |
| ------------------------------------ | -------------------------- |
| `px4_fmu-v3_vtol`                 | Pixhawk 2 (Cube Black)     |
| `px4_fmu-v4_vtol`                 | Pixhawk 3 Pro              |
| `px4_fmu-v4pro_vtol`              | Pixracer                   |
| `px4_fmu-v5_vtol`                 | Pixhawk 4                  |
| `px4_fmu-v5x_vtol`                | Pixhawk 5X                 |
| `px4_fmu-v6c_vtol`                | Pixhawk 6C                 |
| `px4_fmu-v6c_raptor`                 | Pixhawk 6C (Raptor)        |
| `px4_fmu-v6x_multicopter`            | Pixhawk 6X (multicopter)   |
| `auterion_fmu-v6s_vtol`           | Auterion FMU-v6S           |
| `auterion_fmu-v6x_vtol`           | Auterion FMU-v6X           |
| `holybro_durandal-v1`        | Holybro Durandal           |
| `holybro_kakuteh7`           | Holybro Kakute H7          |
| `holybro_kakuteh7v2`         | Holybro Kakute H7 V2       |
| `holybro_pix32v5_vtol`            | Holybro Pix32 V5           |
| `cuav_nora`                  | CUAV Nora                  |
| `cuav_x7pro`                 | CUAV X7 Pro                |
| `cuav_x25-evo`               | CUAV X25 EVO               |
| `cuav_x25-super`             | CUAV X25 Super             |
| `cubepilot_cubeyellow_vtol`       | CubePilot Cube Yellow      |
| `mro_pixracerpro_vtol`            | MRO PixRacer Pro           |
| `mro_x21_vtol`                    | MRO X2.1                   |
| `mro_ctrl-zero-h7_vtol`           | MRO Ctrl Zero H7           |
| `mro_ctrl-zero-h7-oem_vtol`       | MRO Ctrl Zero H7 OEM       |
| `mro_ctrl-zero-f7_vtol`           | MRO Ctrl Zero F7           |
| `mro_ctrl-zero-f7-oem_vtol`       | MRO Ctrl Zero F7 OEM       |
| `mro_ctrl-zero-classic_vtol`      | MRO Ctrl Zero Classic      |
| `3dr_ctrl-zero-h7-oem-revg_vtol`  | 3DR Ctrl Zero H7 OEM RevG  |
| `modalai_fc-v1_vtol`              | ModalAI FC V1              |
| `nxp_fmuk66-v3`              | NXP FMUK66-V3              |
| `nxp_fmuk66-e`               | NXP FMUK66-E               |
| `radiolink_PIX6_vtol`             | Radiolink PIX6             |
| `siyi_n7`                    | SIYI N7                    |
| `sky-drones_smartap-airlink` | Sky-Drones SmartAP Airlink |
| `uvify_core`                 | UVify Core                 |
| `atl_mantis-edu`             | ATL Mantis EDU             |
| `av_x-v1_vtol`                    | AV X-V1                    |
| `narinfc_h7`                 | NarinFC H7                 |
| `thepeach_k1_vtol`                | ThePeach K1                |
| `thepeach_r1_vtol`                | ThePeach R1                |
| `airmind_mindpx-v2_vtol`          | AirMind MindPX V2          |
| `beaglebone_blue`            | BeagleBone Blue            |
| `bluerobotics_navigator`     | BlueRobotics Navigator     |
| `emlid_navio2`               | Emlid Navio2               |
| `px4_raspberrypi`            | Raspberry Pi               |
| `scumaker_pilotpi`           | Scumaker PilotPi           |

::: info
Some boards (e.g., `px4_fmu-v6x_vtol`, `cubepilot_cubeorange`) do not include SIH in their default build due to flash memory constraints.
You can add SIH to any board -- see [Check if SIH is in Firmware](#check-if-sih-is-in-firmware).
:::

## Requirements

- A flight controller with SIH module included in firmware (see [Firmware Builds with SIH](#firmware-builds-with-sih)).
- USB connection for QGroundControl.
- Optional: jMAVSim for 3D visualization via serial link (see [Visualization](#hardware-visualization)).

## Check if SIH is in Firmware

SIH is included in [most default firmware builds](#check-if-sih-is-in-firmware).
To verify, search for `sih` in the parameter list in QGroundControl. If `SIH_*` parameters are available, the module is included.

To add SIH to a custom build, enable it in the board configuration:

```txt
CONFIG_MODULES_SIMULATION_SIMULATOR_SIH=y
```

## Visualization (Optional) {#hardware-visualization}

If you need a visual aid to see what the simulated vehicle is doing on hardware:

### QGroundControl

Connect the flight controller via USB. QGC shows the vehicle on the map view with attitude, position, and telemetry, the same as a real flight.

### jMAVSim (3D Display-Only)

jMAVSim can render a 3D view of the vehicle over a serial connection. No physics are simulated in jMAVSim -- it is display-only.

```sh
./Tools/simulation/jmavsim/jmavsim_run.sh -q -d /dev/ttyACM0 -b 2000000 -o
```

Where `/dev/ttyACM0` is the serial device for the flight controller.
On macOS, this is typically `/dev/tty.usbmodem*`.

## Controlling Actuators

:::warning
If you want to control throttle actuators in SIH, make sure to remove propellers for safety.
:::

In some scenarios, it may be useful to control an actuator while running SIH on hardware. For example, you might want to verify that winches or grippers are functioning correctly by checking the servo responses.

**To enable actuator control in SIH:**

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

   ```sh
   pwm_out start
   ```

   **And to disable it again:**

   ```sh
   pwm_out stop
   ```

## Adding New Airframes (FC)

Airframe configuration for SIH on a flight controller differs from SITL in a few ways:

- Airframe file goes in `ROMFS/px4fmu_common/init.d/airframes` and follows the naming template `${ID}_${model_name}.hil`, where `ID` is the `SYS_AUTOSTART_ID` used to select the airframe, and `model_name` is the airframe model name.
- Add the model name in `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` to generate a corresponding make target.
- Actuators are configured with `HIL_ACT_FUNC*` parameters (not the usual `PWM_MAIN_FUNC*` parameters).
  This is to avoid using the real actuator outputs in SIH.
  Similarly, the bitfield for inverting individual actuator output ranges is `HIL_ACT_REV`, rather than `PWM_MAIN_REV`.

For general airframe setup (SIH parameters, EKF2 tuning), see [Adding New Airframes](index.md#adding-new-airframes) on the main SIH page.

For examples, see the `.hil` airframes in [ROMFS/px4fmu_common/init.d/airframes](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d/airframes).
