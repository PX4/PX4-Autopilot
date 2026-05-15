# Hiwonder 4-Channel Encoder Motor Module

The [Hiwonder 4-Channel Encoder Motor Driver](https://www.hiwonder.com/products/4-channel-encoder-motor-driver) is a small I2C motor controller with integrated encoder feedback for up to four brushed DC motors.
It is well suited to small wheeled rovers (differential, ackermann, or mecanum) where size, weight, and a low channel count make a full-size ESC overkill.

PX4 supports the board via the `hiwonder_emm` I2C driver.

## Features

- Four independent motor outputs with closed-loop speed control from on-board encoders.
- I2C interface (default address `0x34`, 400 kHz).
- Battery voltage telemetry of the supply rail powering the motor outputs.
- Selectable motor types (TT, N20, JGB37-520-12V-110RPM, or open-loop "no encoder").

## Where to Buy

- [Hiwonder — 4-Channel Encoder Motor Driver product page](https://www.hiwonder.com/products/4-channel-encoder-motor-driver)

## Hardware Setup

### Wiring

Connect the four motors (with their encoder leads) to channels `M1`–`M4` on the EMM and supply the motor driver from a battery suitable for the motors used.
Wire the EMM's I2C bus (`SDA`, `SCL`, `GND`) to an external I2C port on the flight controller.
The driver is hard-coded to bus `1` (the first external I2C bus) at address `0x34`, which is the board default.

::: info
The driver auto-detects the four channels but the motor type is selected in firmware.
The shipped default is `JGB37-520-12V-110RPM`; change [`HiwonderEMM.cpp`](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/hiwonder_emm/HiwonderEMM.cpp) (`set_motor_type`) if you use a different model.
:::

## Flight Controller Setup

### Enable the Driver

Set the [HIWONDER_EMM_EN](../advanced_config/parameter_reference.md#HIWONDER_EMM_EN) parameter to `1` and reboot.

The driver is started automatically by the rover startup script ([`rc.rover`](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/rc.rover)), which runs for every ackermann, differential, and mecanum airframe.
It is compiled into builds that enable `CONFIG_DRIVERS_HIWONDER_EMM` in the board configuration.

### Actuator Allocation

The driver exposes a four-channel output group with the `EMM` parameter prefix.
In [Actuator Configuration](../config/actuators.md), assign the rover wheel outputs (e.g. `Motor 1`, `Motor 2`, …) to channels `EMM 1`–`EMM 4` matching how the motors are wired to the EMM.

The output range is fixed in firmware to `0..255` (the EMM's protocol range, internally mapped to a signed `[-128, 127]` speed command — `128` is stop, values below are reverse, values above are forward).
`EMM_DIS{i}` (disarmed) and `EMM_FAIL{i}` (failsafe) per-channel parameters are user-tunable in the standard range.

## Further Information

- Driver source: [`src/drivers/hiwonder_emm`](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/hiwonder_emm)
- [Actuator Configuration](../config/actuators.md)
- [Rovers](../frames_rover/index.md)
