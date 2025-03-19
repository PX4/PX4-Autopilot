# Holybro X500 + Pixhawk4 Build

:::info
Holybro initially supplied this kit with a [Holybro Pixhawk 4](../flight_controller/pixhawk4.md)), but at time of writing this has been upgraded to a [Holybro Pixhawk 6C](../flight_controller/pixhawk6c.md).
This build log is still relevant as the kit assembly is virtually the same, and likely to remain so as the flight controller is upgraded.
:::

This topic provides full instructions for building the kit and configuring PX4 using _QGroundControl_.

## Key information

- **Full Kit:** [Holybro X500 Kit](https://holybro.com/products/px4-development-kit-x500-v2)
- **Flight controller:** [Pixhawk 4](../flight_controller/pixhawk4.md)
- **Assembly time (approx.):** 3.75 hours (180 minutes for frame, 45 minutes for autopilot installation/configuration)

![Full X500 Kit](../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_hero.png)

## Bill of materials

The Holybro [X500 Kit](https://holybro.com/products/px4-development-kit-x500-v2) includes almost all the required components:

- [Pixhawk 4 autopilot](../flight_controller/pixhawk4.md)
- [Holybro M8N GPS](https://holybro.com/collections/gps/products/m8n-gps)
- [Power Management - PM07](../power_module/holybro_pm07_pixhawk4_power_module.md)
- Holybro Motors - 2216 KV880 x4 (superseded - check [spare parts list](https://holybro.com/products/spare-parts-x500-v2-kit) for current version).
- Holybro BLHeli S ESC 20A x4 (superseded - check [spare parts list](https://holybro.com/products/spare-parts-x500-v2-kit) for current version).
- Propellers - 1045 x4 (superseded - check [spare parts list](https://holybro.com/products/spare-parts-x500-v2-kit) for current version).
- Battery Strap
- Power and Radio Cables
- Wheelbase - 500 mm
- Dimensions - 410x410x300 mm
- 433 MHz / 915 MHz [Holybro Telemetry Radio](../telemetry/holybro_sik_radio.md)

Additionally you will need a battery and receiver ([compatible radio system](../getting_started/rc_transmitter_receiver.md)) if you want to control the drone manually.

## 硬件

This section lists all hardware for the frame and the autopilot installation.

| Item                                            | 描述                                                                                                  | Quantity |
| ----------------------------------------------- | --------------------------------------------------------------------------------------------------- | -------- |
| Bottom plate                                    | Carbon fiber (2mm thick)                                                         | 1        |
| Top plate                                       | Carbon fiber (1.5mm thick)                                       | 1        |
| Arm                                             | Carbon fiber tube (Diameter: 16mm length: 200mm) | 4        |
| Landing gear - Vertical pole                    | Carbon fiber tube + engineering plastic                                                             | 2        |
| Landing gear - Cross bar                        | Carbon fiber tube + engineering plastic + foam                                                      | 2        |
| Motor base                                      | Consists of 6 parts and 4 screws 4 nuts                                                             | 4        |
| Slide bar                                       | Diameter: 10mm length: 250mm                                        | 2        |
| Battery mounting board                          | Thickness: 2mm                                                                      | 1        |
| Battery pad                                     | 3mm Silicone sheet black                                                                            | 1        |
| Platform board                                  | Thickness: 2mm                                                                      | 1        |
| Hanger & rubber ring gasket | Inner hole diameter: 10mm black                                                     | 8        |

![X500 Full Package Contents](../../assets/airframes/multicopter/x500_holybro_pixhawk4/whats_inside_x500_labeled.jpg)

### Electronics

| Item Description                                                               | Quantity |
| ------------------------------------------------------------------------------ | -------- |
| Pixhawk4 & Assorted Cables                                 | 1        |
| Pixhawk4 GPS Module                                                            | 1        |
| Power Management PM07 (with pre-soldered ESC power cables)  | 1        |
| Motors 2216 KV880（V2 Update)                                                   | 4        |
| Holybro BLHeli S ESC 20A x4                                                    | 1        |
| 433 MHz / 915 MHz [Holybro Telemetry Radio](../telemetry/holybro_sik_radio.md) | 1        |

### Tools needed

The following tools are used in this assembly:

- 1.5 mm Hex screwdriver
- 2.0 mm Hex screwdriver
- 2.5 mm Hex screwdriver
- 3mm Phillips screwdriver
- 5.5 mm socket wrench or small piler
- Wire cutters
- Precision tweezers

## 组装

Estimate time to assemble is 3.75 hours (180 minutes for frame, 45 minutes for autopilot installation/configuration)

1. Start by assembling the landing gear.
  Unscrew the landing gear screws and insert the vertical pole (figures 1 and 2).

  ![Landing Figure 1: Components](../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_step_1_fig1.jpg)

  _Figure 2_: Landing gear components

  ![Landing Figure 2: Assembled](../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_step_1_fig2.jpg)

  _Figure 2_: Landing gear assembled

2. Then put the 4 arms through the 4 motor bases shown in figure 3.
  Make sure the rods protrude the base slightly and are consistent throughout all 4 arms, and be sure to have the motor wires facing outward.

  ![Attach arms to motor bases](../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_step_2_fig3.png)

  _Figure 3_: Attach arms to motor bases

3. Insert 4 nylon screws and nylon standoffs and attach the power module PM07 to the bottom plate using 4 nylon nuts as shown in Figures 4.

  ![Attach power module](../../assets/airframes/multicopter/x500_holybro_pixhawk4/power_module.jpg)

  _Figure 4_: Attach power module

4. Feed the 4 motor ESCs through each of the arms and connect the 3-wires end to the motors shown in Figure 5.

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_fig17.jpg" width="250" title="Connect motors">

  _Figure 5_: Connect motors

5. Connect the ESCs power wires onto the power module PM07, black->black and red->red, ESC PWM signal wires goes to "FMU-PWM-Out".
  Make sure you connect the motor ESC PWM wires in the correct order.
  Refer to Figure 7 for airframe motor number and connect to the corresponding number on the PM07 board.

  ![ESC power module and signal wiring](../../assets/airframes/multicopter/x500_holybro_pixhawk4/pm07_pwm.jpg)
  _Figure 7_: ESC power module and signal wiring

  The color on top of the motor indicate the spin direction (figure 7-1), black tip is clockwise, and white tip is counter-clockwise.
  Make sure the follow the px4 quadrotor x airframe reference for motor direction (figure 7-2).

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/quadx.png" width="240">

  _Figure 7_: Motor order/direction diagram

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/motor_direction1.jpg" width="400">

  _Figure 7-1_: Motor direction

6. Connect the 10 pin cables to FMU-PWM-in, the 6 pin cables to the PWR1 on the PM07 power module.

  ![Flight controller/Power module PWM and Power connections](../../assets/airframes/multicopter/x500_holybro_pixhawk4/pm07_cable.jpg)

  _Figure 8_: Power module PWM and power wiring

7. If you want to mount the GPS on the top plate, you can now secure the GPS mount onto the top plate using 4 screws and nuts.

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/gpsmount.jpg" width="400" title="Secure GPS mount onto top plate">

  _Figure 9_: Secure GPS mount onto top plate

8. Feed the PM07 cables through the top plate.
  Connect the top and bottom plate by using 4 U-shaped nylon straps, screws, and nuts on each side, ensure that the motor ESC cables are inside the U-shape nylon straps like Figure 10, keep the nut loose.

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/top_plate.jpg" width="300">

  _Figure 10-1_: Feed power module cables through top plate

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/ushape.jpg" width="355" title="Connecting top and bottom plate">

  _Figure 10-2_: Connecting top and bottom plate

9. Push the arm tubes a bit into the frame and make sure the amount of protrusion (red square from Figure 11) are consistent on all 4 arms.
  Ensure all the motors are pointed directly upward, then tighten all the nuts and screws.

  ![Arms 3](../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_fig16.jpg)

10. Put the hanger gaskets into the 4 hangers and mount them onto the bottom plate using 8 hex screws (Figure 11).
  The screw holes are noted by the white arrow in Figure 12.
  We recommend tilting the drone sideway to make the installation easier.

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_fig9.png" width="400" title="Hanger gaskets">

  _Figure 11_: Hanger gaskets

  ![Battery Mount 4](../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_fig10.jpg)

  _Figure 12_: Screw holes

11. Insert the slide bars onto the hanger rings (Figure 13).
  Assemble the battery mount and platform board and mount them onto the slide bars as shown in Figure 14.

  ![Battery Mount 2: Slide bars](../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_fig8.png)

  _Figure 13_: Slide bars

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/battery_mount1.jpg" width="400" title="Battery mount on slide bars">

  _Figure 14_: Battery mount on slide bars

12. Mount the landing gear onto the bottom plate.
  We recommend tilting the drone sideway to make this installation process easier.

  ![Landing Gear](../../assets/airframes/multicopter/x500_holybro_pixhawk4/x500_fig5.jpg)

  _Figure 15_: Landing Gear

13. Use the tape and stick the GPS to the top of the GPS mast and mount the GPS mast.
  Make sure the arrow on the gps is pointing forward (Figure 16).

  <img src="../../assets/airframes/multicopter/x500_holybro_pixhawk4/gps2.jpg" width="400" title="Figure 16: GPS and mast">

  _Figure 16_: GPS and mast

14. Mount the telemetry radio onto the top plate.
  Plug the telemetry cable into `TELEM1` port and GPS module to `GPS MODULE` port on the flight controller.
  Plug the cable from PM07 `FMU-PWM-in` to `I/O-PWM-out`on the FC and PM07 `PWR1` to `POWER1` on the FC, as shown in Figure 17.

  ![Pixhawk 4 wiring 1](../../assets/airframes/multicopter/x500_holybro_pixhawk4/fc_connections.jpg)

  _Figure 17_: Mount telemetry radio/plug in PWM and Power cables to Flight controller.

Please refer to [Pixhawk 4 Quick Start](../assembly/quick_start_pixhawk4.md) for more information.

That's it.
The fully assembled kit is shown below:

![Assembled Kit](../../assets/airframes/multicopter/x500_holybro_pixhawk4/X500_assembled_frame.jpg)

## PX4 配置

:::tip
Full instructions for installing and configuring PX4 can be found in [Basic Configuration](../config/index.md).
:::

_QGroundControl_ is used to install the PX4 autopilot and configure/tune it for the X500 frame.
[Download and install](http://qgroundcontrol.com/downloads/) _QGroundControl_ for your platform.

First update the firmware, airframe, and actuator mappings:

- [Firmware](../config/firmware.md)

- [Airframe](../config/airframe.md)

  You will need to select the _Holybro S500_ airframe (**Quadrotor x > Holybro S500**).

  ![QGroundControl - Select HolyBro X500 airframe](../../assets/airframes/multicopter/s500_holybro_pixhawk4/qgc_airframe_holybro_s500.png)

- [Actuators](../config/actuators.md)
  - You should not need to update the vehicle geometry (as this is a preconfigured airframe).
  - Assign actuator functions to outputs to match your wiring.
  - Test the configuration using the sliders.

Then perform the mandatory setup/calibration:

- [Sensor Orientation](../config/flight_controller_orientation.md)
- [Compass](../config/compass.md)
- [Accelerometer](../config/accelerometer.md)
- [Level Horizon Calibration](../config/level_horizon_calibration.md)
- [Radio Setup](../config/radio.md)
- [Flight Modes](../config/flight_mode.md)

Ideally you should also do:

- [ESC Calibration](../advanced_config/esc_calibration.md)
- [Battery Estimation Tuning](../config/battery.md)
- [Safety](../config/safety.md)

## 调试

Airframe selection sets _default_ autopilot parameters for the frame.
These are good enough to fly with, but it is a good idea to tune the parameters for a specific frame build.

For instructions on how, start from [Autotune](../config/autotune_mc.md).

## Acknowledgements

This build log was provided by the Dronecode Test Flight Team.
