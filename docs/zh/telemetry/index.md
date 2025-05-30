# 数传电台模块

Telemetry Radios can (optionally) be used to provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4. 这使得在飞行过程中调整参数，实时监视遥测信息，更改任务等成为可能。

PX4支持多种类型数传电台：

- [SiK Radio](../telemetry/sik_radio.md) based firmware (more generally, any radio with a UART interface should work).
  - [RFD900 Telemetry Radio](../telemetry/rfd900_telemetry.md)
  - [HolyBro SiK Telemetry Radio](../telemetry/holybro_sik_radio.md)
  - <del>_HKPilot Telemetry Radio_</del> (Discontinued)
  - <del>_3DR Telemetry Radio_</del> (Discontinued)
- [Telemetry Wifi](../telemetry/telemetry_wifi.md)
- [J.Fi Wireless Telemetry Module](../telemetry/jfi_telemetry.md)
- [Microhard Serial Telemetry Radio](../telemetry/microhard_serial.md)
  - [ARK Electron Microhard Serial Telemetry Radio](../telemetry/ark_microhard_serial.md)
  - [Holybro Microhard P900 Telemetry Radio](../telemetry/holybro_microhard_p900_radio.md)
- CUAV Serial Telemetry Radio
  - [CUAV P8 Telemetry Radio](../telemetry/cuav_p8_radio.md)
- XBee Serial Telemetry Radio
  - <del>[HolyBro XBP9X Telemetry Radio](../telemetry/holybro_xbp9x_radio.md)</del> (Discontinued)

PX4 is protocol compatible with [SiK Radio](../telemetry/sik_radio.md) and will generally work out of the box (though you may need to change/use an appropriate connector).

WIFI 数传通常具有更短的范围、更高的数据速率，并且可以更轻松地支持FPV/视频源。
Wifi电台的一个好处是, 您只需为您的车辆购买一个无线电设备（假设地面站已经有WIFI）。

:::info
PX4 does not support connecting an LTE USB module to the flight controller (and sending MAVLink traffic via the Internet).
You can however connect an LTE module to a companion computer and use it to route MAVLink traffic from the flight controller.
For more information see: [Companion Computer Peripherals > Data Telephony](../companion_computer/companion_computer_peripherals.md#data-telephony-lte).
:::

## Allowed Frequency Bands

Radio bands allowed for use with drones differ between continents, regions, countries, and even states.
You should select a telemetry radio that uses a frequency range that is allowed in the areas where you plan on using the drone.

Low power [SiK radios](../telemetry/sik_radio.md), such as the [Holybro Telemetry Radio](../telemetry/holybro_sik_radio.md), are often available in 915 MHz and 433 MHz variants.
While you should check applicable laws in your country/state, broadly speaking 915 MHz can be used in the US, while 433 MHz can be used in EU, Africa, Oceania, and most of Asia.
