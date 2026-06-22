# Holybro SiK Telemetry Radio - Long Range

This Holybro SiK Long Range Telemetry Radio is a small, light, and inexpensive open-source radio platform with an extended range (~20km) compared to the standard model.

This radio is plug-and-play, ready for all Pixhawk Standard and other similar flight controllers, providing the easiest way to set up a telemetry connection between your controller and a ground station.
It uses open-source firmware that has been specially designed to work well with MAVLink packets and to be integrated with PX4, ArduPilot, Mission Planner and QGroundControl.

The radios are available in 915 MHz or 433 MHz versions.
Please purchase the model that is appropriate for your country/region.

![Sik Telemetry Radio - Long Range](../../assets/hardware/telemetry/holybro_sik_longrange/holybro_sik_longrange.jpg)

## 购买渠道

- [Holybro SiK Telemetry Radio - Long Range](https://holybro.com/collections/telemetry-radios/products/sik-telemetry-radio-1w)

## 特性

- 1W maximum RF output and up to 20km range (compared to 100mW/300m for the short range version).
- Open-source SIK firmware
- Plug-n-play for Pixhawk Standard Flight Controllers
- The Easiest way to connect your controller and Ground Station
- Interchangeable air and ground radio
- 6-position JST-GH connector

## 技术规范

- 1 W maximum output power (adjustable) -117 dBm receive sensitivity
- RP-SMA connector
- 2-way full-duplex communication through adaptive TDM UART interface
- Transparent serial link
- MAVLink protocol framing
- Frequency Hopping Spread Spectrum (FHSS) Configurable duty cycle
- Error correction corrects up to 25% of bit errors Open-source SIK firmware
- Configurable through Mission Planner & APM Planner
- FT230X USB to BASIC UART IC
- USB Type C connector
- XT30 power connector for 7~28V DC input

## LEDs Indicators Status

The radios have four status LEDs.
The USB `RX` and `TX` LEDs indicate the status of reception and transmission of the USB port.
The `RADIO` and `ACT` two LED lights indicate the status of the RF circuit.

- USB-TX LED (orange):
  - Blinking: USB port has data transmission
  - Off: USB port has no data transmission
- USB-RX LED (orange):
  - Blinking: USB port has data reception
  - Off: USB port has no data reception
- Radio LED (Green)
  - Blinking: Searching for another radio
  - Solid: Link is established with another radio
- ACT LED (Red)
  - Flashing: Transmitting data
  - Solid: In firmware update mode

![Holybro SiK LongRange LED Indicators](../../assets/hardware/telemetry/holybro_sik_longrange/holybro_sik_longrange_label.png)

## Connecting to Flight Controller

Supply the power (7~28V) to the radio via the XT30 connector.
Use the 6-pin JST-GH connector that comes with the radio to connect the radio to your flight controller's `TELEM1` port.

Note that `TELEM2` can also be used, but you may need to [configure the telemetry port](../peripherals/mavlink_peripherals.md) on some flight controllers.

## Connecting to a PC or Ground Station

First, power the module with a 7~28V DC source.
Then, connect the radio to your Windows PC or Ground Station using a Type-C USB cable.

The necessary drivers should be installed automatically, and the radio will appear as a new “USB Serial Port” in the Windows Device Manager under Ports (COM & LPT).
The Mission Planner's COM Port selection drop-down should also include the newly added COM port.

## Packages Include

### Single Radio

- 1W Radio modules with antennas (1)
- High-gain omnidirectional antenna (1)
- Male Type-C to male Type-C USB cable (1)
- Male XT30 to female XT30 adapter cable (1)
- Male XT30 to female XT60 adapter cable (1)
- JST-GH-6P to JST-GH-6P cable (1) (for Pixhawk Standard FC)
- Rubber damping grommet (3)

![Sik Telemetry Radio - LongRange Package1](../../assets/hardware/telemetry/holybro_sik_longrange/holybro_sik_longrange_include1.png)

### Pair Radios

- 1W Radio modules with antennas (2)
- High-gain omnidirectional antenna (2)
- Male Type-C to male Type-C USB cable (1)
- Male XT30 to female XT30 adapter cable (1)
- Male XT30 to female XT60 adapter cable (1)
- JST-GH-6P to JST-GH-6P cable (1) (for Pixhawk Standard FC)
- Rubber damping grommet (3)

![Sik Telemetry Radio - LongRange Package2](../../assets/hardware/telemetry/holybro_sik_longrange/holybro_sik_longrange_include2.png)
