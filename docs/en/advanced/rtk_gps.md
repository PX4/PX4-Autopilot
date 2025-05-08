# RTK GNSS/GPS (PX4 Integration)

[Real Time Kinematic](https://en.wikipedia.org/wiki/Real_Time_Kinematic) (RTK) provides centimeter-level GPS accuracy.
This page explains how RTK is integrated into PX4.

:::tip
Instructions for _using_ RTK GNSS are provided in [Hardware > RTK GPS](../gps_compass/rtk_gps.md).
:::

## Overview

RTK uses measurements of the phase of the signal's carrier wave, rather than the information content of the signal.
It relies on a single reference station to provide real-time corrections, which can work with multiple mobile stations.

Two RTK GNSS modules and a datalink are required to setup RTK with PX4.
The fixed-position ground-based GPS unit is called the _Base_ and the in-air unit is called the _Rover_.
The Base unit connects to _QGroundControl_ (via USB) and uses the datalink to stream RTCM corrections to the vehicle (using the MAVLink [GPS_RTCM_DATA](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) message).
On the autopilot, the MAVLink packets are unpacked and sent to the Rover unit, where they are processed to get the RTK solution.

The datalink should typically be able to handle an uplink rate of 300 bytes per second (see the [Uplink Datarate](#uplink-datarate) section below for more information).

## Supported RTK GNSS modules

The list of devices that we have tested can be found [in the user guide](../gps_compass/rtk_gps.md#supported-devices).

::: info
Most devices come with two variants, a base and a rover.
Make sure to select the correct variant.
:::

## Automatic Configuration

The PX4 GPS stack automatically sets up the GPS modules to send and receive the correct messages over the UART or USB, depending on where the module is connected (to _QGroundControl_ or the autopilot).

As soon as the autopilot receives `GPS_RTCM_DATA` MAVLink messages, it automatically forwards the RTCM data to the attached GPS module over existing data channels (a dedicated channel for correction data is not required).

::: info
The u-blox U-Center RTK module configuration tool is not needed/used!
:::

::: info
Both _QGroundControl_ and the autopilot firmware share the same [PX4 GPS driver stack](https://github.com/PX4/GpsDrivers).
In practice, this means that support for new protocols and/or messages only need to be added to one place.
:::

### RTCM messages

QGroundControl configures the RTK base station to output the following RTCM3.2 frames, each with 1 Hz, unless otherwise stated:

- **1005** - Station coordinates XYZ for antenna reference point (Base position), 0.2 Hz.
- **1077** - Full GPS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).
- **1087** - Full GLONASS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).
- **1230** - GLONASS code-phase biases.
- **1097** - Full Galileo pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)
- **1127** - Full BeiDou pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)

## Uplink datarate

The raw RTCM messages from the base are packed into a MAVLink `GPS_RTCM_DATA` message and sent over the datalink.
The maximum length of each MAVLink message is 182 bytes. Depending on the RTCM message, the MAVLink message is almost never completely filled.

The RTCM Base Position message (1005) is of length 22 bytes, while the others are all of variable length depending on the number of visible satellites and the number of signals from the satellite (only 1 for L1 units like M8P).
Since at a given time, the _maximum_ number of satellites visible from any single constellation is 12, under real-world conditions, theoretically an uplink rate of 300 B/s is sufficient.

If _MAVLink 1_ is used, a 182-byte `GPS_RTCM_DATA` message is sent for every RTCM message, irrespective of its length.
As a result the approximate uplink requirement is around 700+ bytes per second.
This can lead to link saturation on low-bandwidth half-duplex telemetry modules (e.g. 3DR Telemetry Radios).

If _MAVLink 2_ is used then any empty space in the `GPS_RTCM_DATA message` is removed.
The resulting uplink requirement is about the same as the theoretical value (~300 bytes per second).

:::tip
PX4 automatically switches to MAVLink 2 if the GCS and telemetry modules support it.
:::

MAVLink 2 must be used on low-bandwidth links for good RTK performance. Care must be taken to make sure that the telemetry chain uses MAVLink 2 throughout.
You can verify the protocol version by using the `mavlink status` command on the system console:

```sh
nsh> mavlink status
instance #0:
        GCS heartbeat:  593486 us ago
        mavlink chan: #0
        type:           3DR RADIO
        rssi:           219
        remote rssi:    219
        txbuf:          94
        noise:          61
        remote noise:   58
        rx errors:      0
        fixed:          0
        flow control:   ON
        rates:
        tx: 1.285 kB/s
        txerr: 0.000 kB/s
        rx: 0.021 kB/s
        rate mult: 0.366
        accepting commands: YES
        MAVLink version: 2
        transport protocol: serial (/dev/ttyS1 @57600)
```
