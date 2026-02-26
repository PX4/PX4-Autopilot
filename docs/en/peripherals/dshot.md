# DShot ESCs

DShot is an alternative ESC protocol that has several advantages over [PWM](../peripherals/pwm_escs_and_servo.md) or [OneShot](../peripherals/oneshot.md):

- Reduced latency.
- Increased robustness via a checksum.
- No need for ESC calibration as the protocol uses digital encoding.
- Telemetry feedback is available/supported on some ESCs.
- Can reverse motor spin directions via commands when needed (rather than physically moving wires/re-soldering).
- Other useful commands are supported.

This topic shows how to connect and configure DShot ESCs.

## Supported ESC

[ESCs & Motors > Supported ESCs](../peripherals/esc_motors#supported-esc) has a list of supported ESC (check "Protocols" column for DShot ESC).

## Wiring/Connections {#wiring}

DShot ESC are wired the same way as [PWM ESCs](pwm_escs_and_servo.md).
The only difference is that they can only be connected to the FMU, and usually only to some subset of pins.

::: info
You may want to check the actuator configuration screen to see what pins are available for DShot on your controller before wiring up!
:::

Pixhawk controllers with both an FMU and an IO board usually label them as `AUX` (FMU) and `MAIN` (IO), respectively.
These match the `PWM AUX` and `PWM MAIN` output tabs on the actuator configuration screen.
For these controllers connect the DShot ESC to the `AUX` port.

Controllers that don't have an IO board usually label the (single) output port as `MAIN`, and this is where you will connect your DShot ESC.
If the controller without IO has its own firmware, the actuator assignment will be to the matching `PWM MAIN` outputs.
However if the same firmware is used for hardware with/without the IO board, such as for the Pixhawk 4 and Pixhawk 4 Mini, then actuator assignment tab used is the same in both cases: `PWM AUX` (i.e. not matching the port label `MAIN` in the "mini" case).

## Configuration

:::warning
Remove propellers before changing ESC configuration parameters!
:::

Enable DShot for your required outputs in the [Actuator Configuration](../config/actuators.md).

DShot comes with different speed options: _DShot150_, _DShot300_, and _DShot600_ where the number indicates the speed in kilo-bits/second.
You should set the parameter to the highest speed supported by your ESC (according to its datasheet).

Then connect the battery and arm the vehicle.
The ESCs should initialize and the motors turn in the correct directions.

- If the motors do not spin in the correct direction (for the [selected airframe](../airframes/airframe_reference.md)) you can reverse them in the UI using the **Set Spin Direction** option (this option appears after you select DShot and assign motors).
  You can also reverse motors by sending an [ESC Command](#commands).

## ESC Commands {#commands}

Commands can be sent to the ESC via the [MAVLink shell](../debug/mavlink_shell.md).
See [here](../modules/modules_driver.md#dshot) for a full reference of the supported commands.

The most important ones are:

- Make a motor connected to to FMU output pin 1 beep (helps with identifying motors)

  ```sh
  dshot beep1 -m 1
  ```

- Retrieve ESC information (requires telemetry, see below):

  ```sh
  nsh> dshot esc_info -m 2
  INFO  [dshot] ESC Type: #TEKKO32_4in1#
  INFO  [dshot] MCU Serial Number: xxxxxx-xxxxxx-xxxxxx-xxxxxx
  INFO  [dshot] Firmware version: 32.60
  INFO  [dshot] Rotation Direction: normal
  INFO  [dshot] 3D Mode: off
  INFO  [dshot] Low voltage Limit: off
  INFO  [dshot] Current Limit: off
  INFO  [dshot] LED 0: unsupported
  INFO  [dshot] LED 1: unsupported
  INFO  [dshot] LED 2: unsupported
  INFO  [dshot] LED 3: unsupported
  ```

- Permanently set the spin direction of a motor connected to FMU output pin 1 (while motors are _not_ spinning):
  - Set spin direction to `reversed`:

    ```sh
    dshot reverse -m 1
    dshot save -m 1
    ```

    Retrieving ESC information will then show:

    ```sh
    Rotation Direction: reversed
    ```

  - Set spin direction to `normal`:

    ```sh
    dshot normal -m 1
    dshot save -m 1
    ```

    Retrieving ESC information will then show:

    ```sh
    Rotation Direction: normal
    ```

  ::: info
  - The commands will have no effect if the motors are spinning, or if the ESC is already set to the corresponding direction.
  - The ESC will revert to its last saved direction (normal or reversed) on reboot if `save` is not called after changing the direction.

  :::

## ESC Telemetry

Some ESCs are capable of sending telemetry back to the flight controller through a UART RX port.
These DShot ESCs will have an additional telemetry wire.

The provided telemetry includes:

- Temperature
- Voltage
- Current
- Accumulated current consumption
- RPM values

To enable this feature (on ESCs that support it):

1. Join all the telemetry wires from all the ESCs together, and then connect them to one of the RX pins on an unused flight controller serial port.
1. Enable telemetry on that serial port using [DSHOT_TEL_CFG](../advanced_config/parameter_reference.md#DSHOT_TEL_CFG).

After a reboot you can check if telemetry is working (make sure the battery is connected) using:

```sh
dshot esc_info -m 1
```

:::tip
You may have to configure the per-motor pole count parameters ([`DSHOT_MOT_POL1`–`DSHOT_MOT_POL12`](../advanced_config/parameter_reference.md#DSHOT_MOT_POL1) to get correct RPM values.
The default value for these is 14 poles, which is typical for 5-inch prop motors.
:::

:::tip
[Extended DShot Telemetry (EDT)](#extended-dshot-telemetry-edt) can provide temperature, voltage, and current through the BDShot signal — no serial telemetry wire needed.
:::

:::tip
Not all DSHOT-capable ESCs support `[esc_info]`(e.g. APD 80F3x), even when telemetry is supported and enabled.
The resulting error is:

```sh
ERROR [dshot] No data received. If telemetry is setup correctly, try again.
```

Check manufacturer documentation for confirmation/details.
:::

## Bidirectional DShot (Telemetry)

Bidirectional DShot (BDShot) enables the ESC to send eRPM telemetry back to the flight controller on the same signal wire used for throttle commands — no additional telemetry wire is needed for RPM data.
High-rate eRPM data significantly improves the performance of [Dynamic Notch Filters](../config_mc/filter_tuning.md#dynamic-notch-filters) and enables more precise vehicle tuning.

With [Extended DShot Telemetry (EDT)](#extended-dshot-telemetry-edt) enabled, BDShot can also provide temperature, voltage, and current data.

### Hardware Support

BDShot requires a flight controller with DMA-capable timers.
Any FMU output on a supported timer can be used for BDShot — multiple timers are supported through sequential burst/capture.

Supported processors:

- **STM32H7**: All FMU outputs on DMA-capable timers
- **i.MXRT** (V6X-RT & Tropic): All FMU outputs

::: info
The ESC must be connected to FMU outputs only.
These are labeled `MAIN` on controllers with a single PWM bus, and `AUX` on controllers with both `MAIN` and `AUX` ports (i.e. those with an IO board).
:::

### PX4 Configuration {#bidirectional-dshot-configuration}

BDShot is enabled **per-timer** in the [Actuator Configuration](../config/actuators.md) UI.
Select **BDShot150**, **BDShot300**, or **BDShot600** as the output protocol instead of the corresponding DShot speed.
There is no separate enable parameter — choosing a BDShot protocol activates bidirectional telemetry on that timer's outputs.

The system calculates actual motor RPM from eRPM data using per-motor pole count parameters: `DSHOT_MOT_POL1` through `DSHOT_MOT_POL12` (one per motor output).
The default is 14 poles, which is typical for 5-inch prop motors.
If you are using AM32 ESCs, the motor pole count must also be set in the AM32 firmware configuration (e.g. via the AM32 configurator tool) to match.

### Extended DShot Telemetry (EDT)

EDT extends BDShot by interleaving temperature, voltage, and current data into the eRPM telemetry frames.
This allows ESC health monitoring through the same signal wire, without requiring a separate serial telemetry connection.

To enable EDT:

1. Configure BDShot on the desired outputs (see above).
2. Set `DSHOT_BIDIR_EDT` to `1` and reboot.

The ESC firmware must support EDT (e.g. [AM32](https://github.com/am32-firmware/AM32)).

When both serial telemetry and BDShot/EDT are enabled, the driver merges data from both sources.

## AM32 ESC Settings (EEPROM)

PX4 can read and write AM32 ESC firmware settings (EEPROM) via a ground station, enabling remote ESC configuration without connecting directly to each ESC.

### Requirements

- ESCs running [AM32 firmware](https://github.com/am32-firmware/AM32) with serial telemetry connected ([DSHOT_TEL_CFG](../advanced_config/parameter_reference.md#DSHOT_TEL_CFG))
- `DSHOT_ESC_TYPE` set to `1` (AM32)
- Ground station with ESC EEPROM support (QGroundControl feature in development)
- MAVLink development dialect enabled on the flight controller

### How It Works

PX4 automatically reads the full EEPROM from each ESC on boot.
The ground station can then display individual settings and allow the user to modify them.
Changes are written back to the ESC one byte at a time using the DShot programming protocol.
