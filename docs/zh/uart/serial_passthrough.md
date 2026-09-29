# Serial Passthrough (MAVLink SERIAL_CONTROL)

<Badge type="tip" text="PX4 v1.18" />

Serial Passthrough allows a MAVLink client to read from and write to selected flight controller serial interfaces using the [SERIAL_CONTROL](https://mavlink.io/en/messages/common.html#SERIAL_CONTROL) message.
Typical use cases include: providing a direct serial channel for ESC configuration tools, and debugging serial peripherals over a telemetry link.

Two cases are supported:

- Control of normal ports, such as telemetry or GPS ports.
  This works automatically as long as the [`serialpassthrough` driver](../modules/modules_driver.md#serialpassthrough) is present in firmware.
- Control of ESC signal pins on STM32F7/H7 boards via a bit-bang UART implemented in software.
  This requires additional configuration.

When serial control is enabled/supported, passthrough is automatic.
If `SERIAL_CONTROL` traffic is sent to a supported target, PX4 starts handling that target and returns reply data over MAVLink.
You can also start and stop passthrough manually from the PX4 shell.

## 设备ID

The `SERIAL_CONTROL.device` field selects the target UART that is to be controlled.
The following device IDs are allowed (note that this is a subset of the IDs specified in [SERIAL_CONTROL_DEV](https://mavlink.io/en/messages/common.html#SERIAL_CONTROL_DEV)):

| Device ID | 平台                                                     |
| --------- | ------------------------------------------------------ |
| `0`       | TEL1                                                   |
| `1`       | TEL2                                                   |
| `2`       | GPS1                                                   |
| `3`       | GPS2                                                   |
| `4`       | TEL3                                                   |
| `5`       | TEL4                                                   |
| `20`      | ESC channel 0 ([bitbang](#bitbang)) |
| `21`      | ESC channel 1 (bitbang)             |
| `22`      | ESC channel 2 (bitbang)             |
| `23`      | ESC channel 3 (bitbang)             |
| `24`      | ESC channel 4 (bitbang)             |
| `25`      | ESC channel 5 (bitbang)             |
| `26`      | ESC channel 6 (bitbang)             |
| `27`      | ESC channel 7 (bitbang)             |

The UART device path for each port (e.g. `/dev/ttyS1`) is taken from the board's corresponding Kconfig symbols, such as `CONFIG_BOARD_SERIAL_TEL1`, `CONFIG_BOARD_SERIAL_GPS1`, and so on.
If the used device ID is not configured on the board, the driver logs a warning and rejects the message.

## UART Control

Normal UARTs such as those for telemetry and GPS are targeted using device IDs 0 to 5.

This feature requires only that the [`serialpassthrough` driver](../modules/modules_driver.md#serialpassthrough) is present in firmware (the KConfig key `CONFIG_DRIVERS_SERIALPASSTHROUGH=y` must be set).
Note that the `PASSTHRU_EN` parameter need not be set.

## ESC Channel Mode (Bitbang UART) {#bitbang}

:::tip
This feature is not yet useful because PX4 does not ship a ready-made tool for using serial passthrough with common ESC configuration or firmware flashing tools.
Information on how such a tool might be developed is given below in [Bridge Application](#bridge-application).
:::

Device IDs 20–27 route through a software bit-bang UART on the ESC signal pin rather than a hardware UART.
This is useful for communicating with ESCs that expose a UART telemetry or configuration port on their signal wire (such as BLHeli_32 passthrough, AM32, or ESC configuration tools).

Bitbang UART is implemented using a hardware timer and direct GPIO toggling.
Due to interrupt latency, reliable operation is only guaranteed up to **19200 baud**.

Because only one hardware timer is used, only one ESC channel can be active at a time.
When a request arrives for a different ESC channel, the driver stops the current instance and waits up to 100 ms for it to exit before starting the new one.

Bitbang UART support requires that both `CONFIG_DRIVERS_SERIALPASSTHROUGH=y` and `CONFIG_SERIALPASSTHROUGH_BITBANG=y` are set before building (the timer is selected with `CONFIG_UART_BITBANG_TIMER`, and defaults to `TIM13`).
The `PASSTHRU_EN` parameter must be set to `1` and the device rebooted in order to enable this mode.

### Bridge Application

Developers can create their own bridge application if needed.
This would connect to the vehicle over MAVLink, expose a virtual serial port (e.g. a Unix PTY) to the tool on the host, and translate traffic bidirectionally.
Data written to the PTY would be sent as `SERIAL_CONTROL` messages with `SERIAL_CONTROL_FLAG_RESPOND | SERIAL_CONTROL_FLAG_EXCLUSIVE` set, and incoming `SERIAL_CONTROL` reply messages (with `FLAG_REPLY` set) would be written back to the PTY.

To initialise the passthrough, the bridge should send one `SERIAL_CONTROL` message with the target device ID, the desired UART baud rate in the `baudrate` field, and `count=0` (no payload), then wait approximately 2 seconds for PX4 to spawn the passthrough task before sending real traffic.
For ESC bitbang mode (device IDs 20–27), the bridge must first set `PASSTHRU_EN=1` via `PARAM_SET`, confirm the `PARAM_VALUE` acknowledgement, send `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`, and wait for the FMU heartbeat to return before sending the init message — this ensures the DShot/PWM drivers are not running when the bitbang driver takes over the ESC signal pins.

## 配置

### Firmware Configuration (Build-Time)

The driver is enabled via [Kconfig](../hardware/porting_guide_config.md).
You will need to set the following key in your board:

```text
CONFIG_DRIVERS_SERIALPASSTHROUGH=y # Include the driver
```

If you want to use ESC channel targets you must also set the following keys:

```text
CONFIG_SERIALPASSTHROUGH_BITBANG=y # ESC channel support (NuttX & STM32 only)
CONFIG_UART_BITBANG_TIMER=13       # Timer instance (default TIM13)
```

Then rebuild the firmware.

### PX4 Configuration (ESC targets)

For ESC channel targets (only) you must also set [PASSTHRU_EN](../advanced_config/parameter_reference.md#PASSTHRU_EN) to `1` and reboot the vehicle.

This disables motor control after the reboot (the motor output drivers `dshot` and `pwm_out` are not started).
This is required when you intend to use the ESC bitbang passthrough mode, because the bitbang driver and the DShot/PWM driver cannot share the same ESC signal pins.

:::tip
The parameter auto-resets to `0` on the following reboot, so DShot/PWM output is automatically restored after power cycling.
:::

## 局限

- **Single sender:** only one MAVLink sender is supported at a time.
  A single sender can communicate with multiple ports simultaneously, but replies are always routed back to the MAVLink channel that sent the most recent `SERIAL_CONTROL` message.
  This means that concurrent senders could interfere with each other's replies.
- **ESC bitbang baud rate:** maximum reliable baud rate is 19200.
  Higher rates may work but are not guaranteed.
- **ESC channel exclusivity:** only one ESC bitbang channel can be active at a time.
- **Platform:** bitbang UART is only available on NuttX with STM32F7/H7.
  Requesting ESC bitbang on an unsupported platform logs an error.
- **Buffer size:** each instance has a 1 kB receive and 1 kB transmit buffer.
  Frames larger than 1 kB will be truncated with a warning logged.
