# CyberX-v10 Wiring Quick Start

This quick start guide shows how to power the [cyberx_v10](../flight_controller/cyberx_v10.md) flight controller and connect its most important peripherals.

## Wiring Chart Overview

The image below shows how to connect the most important sensors and peripherals (except the motor and servo outputs).
We'll go through each of these in detail in the following sections.

![block diagram](../../assets/flight_controller/cyberx_v10/cyberx_v10_block_diagram.png)

| Interface     | **Function**                                                                                                                                                                                      |
| :------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| POWER 1       | Connect SMbus (ADC) power module                                                                                                                                                                  |
| POWER 2       | Connect SMbus (I2C) power module                                                                                                                                                                  |
| GPS (1,2)     | Connect GPS/RTK  module                                                                                                                |
| UART (5,8)    | User customizable                                                                                                                                                                                 |
| TELEM (1,2)   | Connect telemetry or MAVLink devices                                                                                                                                                              |
| I2C (3,4)   | Connect I2C peripherals, such as the airspeed meter MS4525DO module, etc.                                                                                                                                                              |
| TF CARD       | SD card for log storage (card pre-inserted in factory).                                                                                                                                           |
| M1~M8         | IO PWM output (for connecting to ESC and Servo)                                                                                                                                                   |
| M9~M16        | FMU PWM output. Can be defined as PWM/GPIO; supports dshot; used to connect camera shutter/hot shoe, servo, etc.                                                                                  |
| USB           | Connect to a computer for communication between the flight controller and the computer, such as loading firmware.                                                                                 |
| CAN1/CAN2     | Connect Dronecan/UAVCAN devices.                                                                                                                                                 |
| SBUS/RSSI     | Includes SBUS, RSSI signal input interface, SBUS_IN pin for SBUS remote control receiver, RSSI_IN pin for signal strength return module |
| PPM           | Connecting the RCin Receiver                                                                                                                                                                    |
| ETH           | Ethernet interface. Connect Ethernet devices such as task computers                                                                                                                               |
| AD&IO         | There are two analog inputs (ADC3.3/ADC6.6); generally not used                                                                                                                                   |

## Vehicle Front

::: info
If the controller cannot be mounted in the recommended/default orientation (e.g. due to space constraints) you will need to configure the autopilot software with the orientation that you actually used: [Flight Controller Orientation](../config/flight_controller_orientation.md).
:::

![front](../../assets/flight_controller/cyberx_v10/cyberx_v10_front.png)

## GPS + Compass + Buzzer + Safety Switch + LED

The CyberX-v10 can be purchased with a GNSS modules (6-pin*2 connector) and should be connected to the **GPS1** port and the **SW+BUZ** port.
These GNSS modules feature an integrated compass, safety switch, buzzer and LEDs.

If you need to use assisted GPS, connect to the **GPS2** port.

The GPS/compass should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction markings towards the front of the vehicle (separating the compass from other electronics will reduce interference).

::: info
The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety press and hold the safety switch for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle (this can be useful if, for whatever reason, you are unable to disarm the vehicle from your remote control or ground station).
:::

## Radio Control

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

- Spektrum/DSM receivers connect to the **Uart8** input port.
- PPM receivers connect to the **RCin** input pin.
- SBUS receivers connect to the **S.Bin** input port.

For more information about selecting a radio system, receiver compatibility, and binding your transmitter/receiver pair, see: [Remote Control Transmitters & Receivers](../getting_started/rc_transmitter_receiver.md).

## Power

CyberX-v10 is equipped with a ADC and I2C PMU lite module that supports 2~20s lithium battery.
Connect the 6pin connector of the module to the flight control **PWR1** or **PWR2** interface.

CyberX-v10 is configured by default for an analog power monitor on **PWR1** port.and the board on **PWR2** port receives digital signal from I2C PMU lite power module for voltage, current and remaining battery data, the VCC line must provide at least 3A continuous current and should default to 5.2V.
A lower voltage of 5V is still acceptable but discouraged.

## Telemetry (Radio) System

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TEL1**/**TEL2** port as shown below (if connected to **TEL1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

## SD Card

SD cards are highly recommended as they are required for [recording and analyzing flight details](../getting_started/flight_reporting.md), running tasks and using UAVCAN bus hardware.
An SD card is already installed on CyberX-v10 when it leaves the factory.

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

## Motors/Servo

Motors/servos are connected to the **M1~M8** (**MAIN**) and **M9~M16** (**AUX**) ports in the order specified for your vehicle in the [Airframe Reference](../airframes/airframe_reference.md).

::: info
The **MAIN** outputs in the PX4 firmware are mapped to the CyberX-v10's M1~M8 ports (from IO), while the **AUX** are mapped to the M9~M16 ports (from the FMU).
For example, **MAIN1** maps to M1 pin and **AUX1** maps to M9 pin.
This reference lists the output port to motor/servo mapping for all supported air and ground frames (if your frame is not listed in the reference then use a "generic" airframe of the correct type).
:::

:::warning
The mapping is not consistent across frames (e.g. you can't rely on the throttle being on the same output for all plane frames).
Make sure to use the correct mapping for your vehicle.
:::

## Servo Power Supply

CyberX-v10 does not supply power to the servos.
If using a plane or rover, an external BEC (e.g., BEC-equipped ESC or separate 5V BEC) needs to be connected to any of the power (+) pins in M1~M16 to drive the servos.

::: info
The power rail voltage must be appropriate for the servo being used!
:::

## Other Peripherals

The wiring and configuration of optional/less common components is covered within the topics for individual [peripherals](../peripherals/index.md).

## Pinouts

![cyberx_v10 Pinout](../../assets/flight_controller/cyberx_v10/cyberx_v10_pinout.png)

## Configuration

General configuration information is covered in: [Autopilot Configuration](../config/index.md).

QuadPlane specific configuration is covered here: [QuadPlane VTOL Configuration](../config_vtol/vtol_quad_configuration.md)

## Further information

- [cyberx_v10](../flight_controller/cyberx_v10.md) (PX4 Doc Overview page)
- [CyberCraft International Limited](https://int.woocoo.vip/)
