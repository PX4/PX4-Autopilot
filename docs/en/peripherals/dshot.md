# DShot ESCs

DShot is an alternative ESC protocol that has several advantages over [PWM](../peripherals/pwm_escs_and_servo.md) or [OneShot](../peripherals/oneshot.md):

- Reduced latency.
- Increased robustness via a checksum.
- No need for ESC calibration as the protocol uses digital encoding.
- Telemetry feedback is available/supported on some ESCs.
- Can reverse motor spin directions via commands when needed (rather than physically moving wires/re-soldering).
- Other useful commands are supported.

This topic shows how to connect and configure DShot ESCs.

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
You may have to configure [MOT_POLE_COUNT](../advanced_config/parameter_reference.md#MOT_POLE_COUNT) to get the correct RPM values.
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

<Badge type="tip" text="PX4 v1.16" />

Bidirectional DShot is a protocol that can provide telemetry including: high rate ESC RPM data, voltage, current, and temperature with a single wire.

The PX4 implementation currently enables only ESC RPM (eRPM) data collection from each ESC at high frequencies.
This telemetry significantly improves the performance of [Dynamic Notch Filters](../config_mc/filter_tuning.md#dynamic-notch-filters) and enables more precise vehicle tuning.

::: info
The [ESC Telemetry](#esc-telemetry) described above is currently still necessary if you want voltage, current, or temperature information.
It's setup and use is independent of bidirectional DShot.
:::

### Hardware Setup

The ESC must be connected to FMU outputs only.
These will be labeled `MAIN` on flight controllers that only have one PWM bus, and `AUX` on controllers that have both `MAIN` and `AUX` ports (i.e. FCs that have an IO board).

:::warning **Limited hardware support**
This feature is only supported on flight controllers with the following processors:

- STM32H7: First four FMU outputs
  - Must be connected to the first 4 FMU outputs, and these outputs must also be mapped to the same timer.
  - [KakuteH7](../flight_controller/kakuteh7v2.md) is not supported because the outputs are not mapped to the same timer.
- [i.MXRT](../flight_controller/nxp_mr_vmu_rt1176.md) (V6X-RT & Tropic): 8 FMU outputs.

No other boards are supported.
:::

### Configuration {#bidirectional-dshot-configuration}

To enable bidirectional DShot, set the [DSHOT_BIDIR_EN](../advanced_config/parameter_reference.md#DSHOT_BIDIR_EN) parameter.

The system calculates actual motor RPM from the received eRPM data using the [MOT_POLE_COUNT](../advanced_config/parameter_reference.md#MOT_POLE_COUNT) parameter.
This parameter must be set correctly for accurate RPM reporting.
