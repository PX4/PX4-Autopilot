# PPS Time Synchronization (PX4 Integration)

[Pulse Per Second](https://en.wikipedia.org/wiki/Pulse-per-second_signal) (PPS) time synchronization provides high-precision timing for GNSS receivers.
This page explains how PPS is integrated into PX4 and how to configure it.

## Overview

PPS (Pulse Per Second) is a timing signal provided by GNSS receivers that outputs an electrical pulse once per second, synchronized to UTC time.
The PPS signal provides a highly accurate timing reference that PX4 can use to:

- Refine GNSS time measurements and compensate for clock drift
- Provide precise UTC timestamps for camera capture events (for photogrammetry and mapping applications)
- Enable offline position refinement through accurate time correlation

## Supported Hardware

PPS time synchronization can be supported on flight controllers that have a hardware timer input pin that can be configured for PPS capture, by [enabling the PPS capture driver](#enable-pps-driver-in-board-configuration) in the board configuration.

Supported boards include (at time of writing):

- [Ark FMUv6x](../flight_controller/ark_v6x.md)
- Auterion FMUv6x
- Auterion FMUv6s

## Setup

### Enable PPS Driver in Board Configuration

The [PPS capture driver](../modules/modules_driver.md#pps-capture) must be enabled in the board configuration.
This is done by adding the following to your board's configuration:

```ini
CONFIG_DRIVERS_PPS_CAPTURE=y
```

### Configure PPS Parameters

The configuration varies depending on your flight controller hardware.

#### FMUv6X

For FMUv6X-based flight controllers, configure PWM AUX Timer 3 and Function 9:

```sh
param set PWM_AUX_TIM3 -2
param set PWM_AUX_FUNC9 2064
param set PPS_CAP_ENABLE 1
```

#### FMUv6S

For FMUv6S-based flight controllers, configure PWM MAIN Timer 3 and Function 10:

```sh
param set PWM_MAIN_TIM3 -2
param set PWM_MAIN_FUNC10 2064
param set PPS_CAP_ENABLE 1
```

### Wiring

The wiring configuration depends on your specific flight controller.

#### Skynode X (FMUv6x)

Connect the PPS signal from your GNSS module to the flight controller using the 11-pin or 6-pin GPS connector:

For detailed pinout information, refer to:

- [Skynode GPS Peripherals - Pinouts](https://docs.auterion.com/hardware-integration/skynode/peripherals/gps#pinouts)

#### Skynode S (FMUv6S)

For FMUv6S, you need to route the PPS signal separately:

1. Connect your GNSS module using the standard 6-pin GPS connector: [Skynode S GPS Interface](https://docs.auterion.com/hardware-integration/skynode-s/interfaces#gps)
2. Connect the PPS signal from your GNSS module to the **PPM_IN** pin: [Skynode S Extras 1 Interface](https://docs.auterion.com/hardware-integration/skynode-s/interfaces#extras-1)

#### ARK Jetson Carrier Board (FMUv6x)

For ARK FMUv6X on the Jetson carrier board:

1. Connect your GNSS module using either the 10-pin or 6-pin GPS connector: [ARK PAB GPS1 Interface](../flight_controller/ark_pab#gps1)
2. Connect the PPS signal to the **FMU_CAP** pin: [ARK PAB ADIO Interface](../flight_controller/ark_pab.md#adio)

## Verification

After configuring PPS, you can verify that it is working correctly:

1. Connect to the [PX4 System Console](../debug/system_console.md) (via MAVLink shell or serial console).
2. Wait for GNSS fix.
3. Check the PPS capture status to confirm it is up and running:

   ```sh
   pps_capture status
   ```

4. You can also check the [PpsCapture](../msg_docs/PpsCapture.md) uORB topic

   ```sh
   listener pps_capture
   ```

   Where you should see: `timestamp`, `rtc_timestamp`, and `pps_rate_exceeded_counter`.

### PPS Capture Driver

The PPS capture driver is located in `src/drivers/pps_capture` and uses hardware timer input capture to precisely measure the arrival time of each PPS pulse.

Key features:

- Sub-microsecond pulse capture precision (hardware-dependent)
- Automatic drift calculation and compensation
- Integration with the GNSS driver for refined time stamping

See also:

- [PPS Capture Driver Documentation](../modules/modules_driver.md#pps-capture)
- [PpsCapture Message](../msg_docs/PpsCapture.md)

### Time Synchronization Flow

1. GNSS module sends position/time data at ~1-20 Hz.
2. GNSS module outputs PPS pulse at 1 Hz, precisely aligned to UTC second boundary.
3. PPS capture driver measures the exact time of the PPS pulse arrival using hardware timer.
4. Driver calculates the offset between GNSS time (from UART data) and autopilot clock (from PPS measurement).
5. This offset is used to correct GNSS timestamps and improve sensor fusion accuracy.

The PPS signal provides much higher temporal precision than the transmitted time data, which has latency and jitter from serial communication.

::: warning
If the PPS driver does not sending any data for 5 seconds (despite having `PPS_CAP_ENABLE` set to 1), the `EKF2_GPS_DELAY` will be used instead for estimating the latency.
:::
