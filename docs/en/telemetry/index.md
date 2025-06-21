# Telemetry Radios/Modems

Telemetry Radios can (optionally) be used to provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4. This makes it possible to tune parameters while a vehicle is in flight, inspect telemetry in real-time, change a mission on the fly, etc.

PX4 supports a number of types of telemetry radios:

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

WiFi telemetry typically has shorter range, higher data rates, and makes it easier to support FPV/video feeds.
One benefit of WiFi radios is that you only need to purchase a single radio unit for your vehicle (assuming the ground station already has WiFi).

::: info
PX4 does not support connecting an LTE USB module to the flight controller (and sending MAVLink traffic via the Internet).
You can however connect an LTE module to a companion computer and use it to route MAVLink traffic from the flight controller.
For more information see: [Companion Computer Peripherals > Data Telephony](../companion_computer/companion_computer_peripherals.md#data-telephony-lte).
:::

## Allowed Frequency Bands

Radio bands allowed for use with drones differ between continents, regions, countries, and even states.
You should select a telemetry radio that uses a frequency range that is allowed in the areas where you plan on using the drone.

Low power [SiK radios](../telemetry/sik_radio.md), such as the [Holybro Telemetry Radio](../telemetry/holybro_sik_radio.md), are often available in 915 MHz and 433 MHz variants.
While you should check applicable laws in your country/state, broadly speaking 915 MHz can be used in the US, while 433 MHz can be used in EU, Africa, Oceania, and most of Asia.
