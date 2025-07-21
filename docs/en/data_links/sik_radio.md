# SiK Radio Integration

[SiK radio](https://github.com/LorenzMeier/SiK) is a collection of firmware and tools for telemetry radios.

Information about _using_ SiK Radio can be found in [Peripheral Hardware > Telemetry > SiK Radio](../telemetry/sik_radio.md)

The ("developer") information below explains how to build SiK firmware from source and configure it using AT commands.

## Supported Radio Hardware

The SiK repository includes bootloaders and firmware for the following telemetry radios (2020-02-25):

- HopeRF HM-TRP
- HopeRF RF50-DEMO
- RFD900
- RFD900a
- RFD900p
- RFD900pe
- RFD900u
- RFD900ue

::: info
The SiK repository does not currently firmware for RFD900x or RFD900ux telemetry radios.
In order to update firmware on these radios (for instance, in order to support MAVLink v2.0), the following process is suggested:

1. Download the appropriate firmware from the [RFDesign website](https://files.rfdesign.com.au/firmware/).
1. On a Windows PC, download and install [RFD Modem Tools](https://files.rfdesign.com.au/tools/).
1. Use the RFD Modem Tools GUI to upload the firmware to your RFD900x or RFD900ux telemetry radio.

:::

## Build Instructions

You will need to install the required 8051 compiler, as this is not included in the default PX4 Build toolchain.

### Mac OS

Install the toolchain:

```sh
brew install sdcc
```

Build the image for the standard SiK Radio / 3DR Radio:

```sh
git clone https://github.com/LorenzMeier/SiK.git
cd SiK/Firmware
make install
```

Upload it to the radio \(**change the serial port name**\):

```
tools/uploader.py --port /dev/tty.usbserial-CHANGETHIS dst/radio~hm_trp.ihx
```

## Configuration Instructions

The radio supports AT commands for configuration.

```sh
screen /dev/tty.usbserial-CHANGETHIS 57600 8N1
```

Then start command mode:

::: info
DO NOT TYPE ANYTHING ONE SECOND BEFORE AND AFTER
:::

```sh
+++
```

List the current settings:

```sh
ATI5
```

Then set the net ID, write settings and reboot radio:

```sh
ATS3=55
AT&W
ATZ
```

::: info
You might have to power-cycle the radio to connect it to the 2nd radio.
:::
