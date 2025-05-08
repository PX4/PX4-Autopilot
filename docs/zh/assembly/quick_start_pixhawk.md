# Pixhawk快速使用指导

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://store.mrobotics.io/) for hardware support or compliance issues.
:::

This quick start guide shows how to power the _3DR Pixhawk_ flight controller and connect its most important peripherals.

![Pixhawk Image](../../assets/flight_controller/pixhawk1/pixhawk_logo_view.jpg)

:::info
The [3DR Pixhawk](../flight_controller/pixhawk.md) is no longer available from 3DR.
Other flight controllers based on the [Pixhawk FMUv2 architecture](../flight_controller/pixhawk_series.md) are available from other companies (these share the same connections, outputs, functions, etc. and are wired in a similar way).
:::

## 接线图概述

The image below shows standard Pixhawk connections (excepting the motor and servo outputs).
We'll go through each main part in the following sections.

![Pixhawk Wiring Overview](../../assets/flight_controller/pixhawk1/pixhawk_wiring_overview.jpg)

:::info
More detailed wiring information is [shown below](#detailed-wiring-infographic-copter).
:::

## 飞控的安装和方向

The _Pixhawk_ should be mounted on the frame using vibration-damping foam pads (included in the kit).
It should be positioned as close to your vehicle’s center of gravity as possible, oriented top-side up with the arrow points towards the front of the vehicle.

![Pixhawk mounting and orientation](../../assets/flight_controller/pixhawk1/pixhawk_3dr_mounting_and_foam.jpg)

:::info
If the controller cannot be mounted in the recommended/default orientation (e.g. due to space constraints) you will need to configure the autopilot software with the orientation that you actually used: [Flight Controller Orientation](../config/flight_controller_orientation.md).
:::

## 蜂鸣器与安全开关

Connect the included buzzer and safety switch as shown below (these are mandatory).

![Pixhawk mounting and orientation](../../assets/flight_controller/pixhawk1/pixhawk_3dr_buzzer_and_safety_switch.jpg)

## GPS + Compass

Attach a GPS (required) to the GPS port using the 6-wire cable supplied in the kit. Optionally attach a compass to the I2C port using a 4-wire cable (the Pixhawk has an internal compass, which can be used if necessary).

:::info
The diagram shows a combined GPS and Compass.
The GPS/Compass should be mounted on the frame as far away from other electronics as possible, with the direction marker towards the front of the vehicle (separating the compass from other electronics will reduce interference).
:::

![Connect compass/GPS to Pixhawk](../../assets/flight_controller/pixhawk1/pixhawk_3dr_compass_gps.jpg)

## 电源

Connect the output of a _Power module_ (PM) to the **POWER** port using a 6-wire cable as shown. The PM input will be connected to your LiPo battery, while the main output will supply vehicle ESCs/motors (possibly via a power distribution board).

The power module supplies the flight controller with power from the battery and also sends information about the analog current and voltage supplied via the module (including both power to the flight controller and to motors etc).

![Pixhawk - Power Module](../../assets/flight_controller/pixhawk1/pixhawk_3dr_power_module.jpg)

:::warning
The power module supplies the flight controller itself, but cannot power servos and other hardware connected to the controller's output ports (rail). For copter this does not matter because the motors are separately powered.
:::

For planes and VTOL the output rail will need to be separately powered in order to drive servos for rudders, elevons etc. Often the main pusher/puller motor uses an ESC with an integrated [BEC](https://en.wikipedia.org/wiki/Battery_eliminator_circuit) that can be connected to the Pixhawk output rail. If not, you will need to setup a 5V BEC to connect to one of the free Pixhawk ports (without power, the servos will not work).

<!-- It would be good to have real example of this powering -->

## 遥控器

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The instructions below show how to connect the different types of receivers to Pixhawk:

- Spektrum and DSM receivers connect to the **SPKT/DSM** input.
  ![Pixhawk - Radio port for Spektrum receivers](../../assets/flight_controller/pixhawk1/pixhawk_3dr_receiver_spektrum.jpg)

- PPM-SUM and S.BUS receivers connect to the **RC** ground, power and signal pins as shown.
  ![Pixhawk - Radio port for PPM/S.BUS receivers](../../assets/flight_controller/pixhawk1/pixhawk_3dr_receiver_ppm_sbus.jpg)

- PPM and PWM receivers that have an _individual wire for each channel_ must connect to the **RC** port _via a PPM encoder_ [like this one](http://www.getfpv.com/radios/radio-accessories/holybro-ppm-encoder-module.html) (PPM-Sum receivers use a single signal wire for all channels).

For more information about selecting a radio system, receiver compatibility, and binding your transmitter/receiver pair, see: [Remote Control Transmitters & Receivers](../getting_started/rc_transmitter_receiver.md).

## Telemetry Radios (Optional)

Telemetry radios may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission). One radio must be connected to your vehicle as shown below. The other is connected to your ground station computer or mobile device (usually by USB).

![Pixhawk/Telemetry Radio](../../assets/flight_controller/pixhawk1/pixhawk_3dr_telemetry_radio.jpg)

<!-- what configuration is required once you've set up a radio) -->

## 电机

The mappings between MAIN/AUX output ports and motor/servos for all supported air and ground frames are listed in the [Airframe Reference](../airframes/airframe_reference.md).

:::warning
The mapping is not consistent across frames (e.g. you can't rely on the throttle being on the same output for all plane frames).
Make sure to use the correct mapping for your vehicle.
:::

:::tip
If your frame is not listed in the reference then use a "generic" airframe of the correct type.
:::

:::info
The output rail must be separately powered, as discussed in the [Power](#power) section above.
:::

<!-- INSERT image of the motor AUX/MAIN ports? -->

## 其它外设

The wiring and configuration of other components is covered within the topics for individual [peripherals](../peripherals/index.md).

## 配置

General configuration information is covered in: [Autopilot Configuration](../config/index.md).

QuadPlane specific configuration is covered here: [QuadPlane VTOL Configuration](../config_vtol/vtol_quad_configuration.md)

<!-- what about config of other vtol types and plane. Do the instructions in these ones above apply for tailsitters etc? -->

## 详细接线示意图(直升机)。

![QuadCopter Pixhawk Wiring Infographic](../../assets/flight_controller/pixhawk1/pixhawk_infographic2.jpg)

## 更多信息

- [Pixhawk Series](../flight_controller/pixhawk_series.md)
- [3DR Pixhawk](../flight_controller/pixhawk.md)
