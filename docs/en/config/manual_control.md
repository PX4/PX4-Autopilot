# Manual Control

Pilots can control a vehicle manually using either a [Radio Control (RC) System](../getting_started/rc_transmitter_receiver.md) or a [Joystick/Gamepad](../config/joystick.md) controller connected via QGroundControl.
PX4 also supports using RC and/or multiple Joysticks, with fallback from one type to the other.

![Taranis X9D Transmitter](../../assets/hardware/transmitters/frsky_taranis_x9d_transmitter.jpg) <img src="../../assets/peripherals/joystick/micronav.jpg" alt="Photo of MicroNav, a ground controller with integrated joysticks" width="400px">

## Overview

_Joystick_ setups use QGroundControl to encode the control information from a "standard" computer gaming joystick into [MAVLink messages](https://mavlink.io/en/services/manual_control.html) that are sent to the vehicle over the (shared) telemetry radio channel.
They are often used in integrated GCS/manual control systems because it is cheaper and easier to integrate a joystick than a separate radio system.
For most applications there is no practical benefit to using RC provided your telemetry channel has a high enough bandwidth/low latency.
They are also perfect for flying the PX4 simulator, because you can plug them directly into your ground control computer and start flying.

_RC systems_ use a dedicated ground-based radio transmitter and vehicle-based receiver for sending control information.
They offer lower latency, and are very highly recommended when first tuning/testing a new frame design, when flying racers/acrobatically, and in other cases where low latency is important.
Note that they usually require significantly more configuration and calibration, much of which may be brand or model-specific.

::: info
PX4 does not _require_ a manual control system for autonomous flight modes.
:::

## PX4 Configuration

::: tip
This section explains how to configure the PX4 to choose and prioritise various manual control sources (other configuration is covered in the guides for each type of manual control).
:::

If you only have one manual control system, either RC or Joystick, then by default no manual control selection is required.
In this case PX4 locks to the first valid manual control source it detects and uses that source until the vehicle is rebooted.

If you have an RC system and/or one or more Joysticks then you can use the [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) parameter to control what source(s) are used, and their priority in the case of fallbacks ([parameters can be set](../advanced_config/parameters.md#finding-a-parameter) using QGC):

- `0`: RC only.
- `1`: MAVLink only.
- `2`: RC or MAVLink with fallback (switches if current source becomes invalid).
- `3`: RC or MAVLink keep first (locks to the first valid source until reboot).
- `4`: Disable manual control (ignores all sources).
- `5`: RC priority, then MAVLink (lower instance before higher) — `RC > MAV 1 > MAV 2`
- `6`: MAVLink priority (lower instance before higher), then RC — `MAVL 1 > MAVL 2 > RC`
- `7`: RC priority, then MAVLink (higher instance before lower) — `RC > MAVL 2 > MAVL 1`
- `8`: MAVLink priority (higher instance before lower), then RCL — `MAVL 2 > MAVL 1 > RC`

RC calibration is required and RC checks are run for any option that uses RC (so not for `MAVLink only` or `Disable manual control`).
When using priority sources, sources are evaluated as soon as they becomes valid and may trigger an immediate switch (if higher priority than other sources).
The [MAVLink instance](../peripherals/mavlink_peripherals.md#mavlink-instances) refers an instance assigned to a serial port, such as [MAV_0_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG).

<!-- Switching
Info on manual control loss vs Data loss and error fallback.
Is no RC calibration a criteria for validity?
-->

## See Also

- [Radio Control (RC)](../getting_started/rc_transmitter_receiver.md)
- [Joysticks](../config/joystick.md)
