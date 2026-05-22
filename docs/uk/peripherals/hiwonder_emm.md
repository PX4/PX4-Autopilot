# Hiwonder 4-Channel Encoder Motor Module

The [Hiwonder 4-Channel Encoder Motor Driver](https://www.hiwonder.com/products/4-channel-encoder-motor-driver) is a small I2C motor controller with integrated encoder feedback for up to four brushed DC motors.
It is well suited to small wheeled rovers (differential, ackermann, or mecanum) where size, weight, and a low channel count make a full-size ESC overkill.

PX4 supports the board via the [`hiwonder_emm`](../modules/modules_driver#hiwonder-emm) I2C driver.

## Функції

- Four independent motor outputs with closed-loop speed control from on-board encoders.
- I2C interface (default address `0x34`, 400 kHz).
- Battery voltage telemetry of the supply rail powering the motor outputs.
- Selectable motor types (TT, N20, JGB37-520-12V-110RPM, or open-loop "no encoder").

## Де купити

- [Hiwonder — 4-Channel Encoder Motor Driver product page](https://www.hiwonder.com/products/4-channel-encoder-motor-driver)

## Налаштування програмного забезпечення

### Підключення

Connect the four motors (with their encoder leads) to channels `M1`–`M4` on the EMM and supply the motor driver from a battery suitable for the motors used.
Wire the EMM's I2C bus (`SDA`, `SCL`, `GND`) to an external I2C port on the flight controller.
The driver is hard-coded to bus `1` (the first external I2C bus) at address `0x34`, which is the board default.

:::info
The driver auto-detects the four channels but the motor type is selected in firmware.
The shipped default is `JGB37-520-12V-110RPM`; change [`HiwonderEMM.cpp`](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/hiwonder_emm/HiwonderEMM.cpp) (`set_motor_type`) if you use a different model.
:::

## Збірка прошивки

To use the EMM, the `hiwonder_emm` driver must be compiled into the firmware and started on boot.

1. Add the following line to the `rover.px4board` file of your board (for Skynode S this would be in [boards/auterion/fmu-v6s/rover.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/auterion/fmu-v6s/rover.px4board)) so the driver is compiled in:

   ```txt
   CONFIG_DRIVERS_HIWONDER_EMM=y
   ```

2. Make sure the driver is started on boot when the [HIWONDER_EMM_EN](../advanced_config/parameter_reference.md#HIWONDER_EMM_EN) parameter is `1`.

   For rover airframes (ackermann, differential, mecanum) this is already done by the shared rover startup script [`rc.rover`](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/rc.rover), which contains:

   ```sh
   if param compare HIWONDER_EMM_EN 1
   then
      hiwonder_emm start
   fi
   ```

   For other airframes that should use the EMM, add the same block to your board's `rc.board_sensors` file (for Skynode S this would be in [boards/auterion/fmu-v6s/init/rc.board_sensors](https://github.com/PX4/PX4-Autopilot/blob/main/boards/auterion/fmu-v6s/init/rc.board_sensors)).

3. Build and flash the firmware for your board.

:::info
Many flight controllers already enable `CONFIG_DRIVERS_HIWONDER_EMM=y` in their `rover.px4board` configuration, so the default rover build is sufficient and step 1 is not required.
:::

## Налаштування польотного контролера

### Enable the Driver

Set the [HIWONDER_EMM_EN](../advanced_config/parameter_reference.md#HIWONDER_EMM_EN) parameter to `1` and reboot.
On the next boot the driver is started by the startup script (see [Building the Firmware](#building-the-firmware)).

### Actuator Allocation

The driver exposes a four-channel output group with the `EMM` parameter prefix.
In [Actuator Configuration](../config/actuators.md), assign the rover wheel outputs (e.g. `Motor 1`, `Motor 2`, …) to channels `EMM 1`–`EMM 4` matching how the motors are wired to the EMM.

The output range is fixed in firmware to `0..255` (the EMM's protocol range, internally mapped to a signed `[-128, 127]` speed command — `128` is stop, values below are reverse, values above are forward).
`EMM_DIS{i}` (disarmed) and `EMM_FAIL{i}` (failsafe) per-channel parameters are user-tunable in the standard range.

## Підтримувані транспортні засоби

The EMM is used on the reference Hiwonder rover frames:

- `50002` — Hiwonder differential rover
- `51003` — Hiwonder ackermann rover
- `52001` — Hiwonder mecanum rover

See [Rovers](../frames_rover/index.md) for vehicle-side configuration.

## Подальша інформація

- Driver source: [`src/drivers/hiwonder_emm`](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/hiwonder_emm)
- [Actuator Configuration](../config/actuators.md)
- [Марсохід](../frames_rover/index.md)
