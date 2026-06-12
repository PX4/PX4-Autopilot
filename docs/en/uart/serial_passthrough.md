# Serial Passthrough (MAVLink SERIAL_CONTROL)

Serial Passthrough allows a MAVLink client to read from and write to selected FC serial interfaces using [SERIAL_CONTROL](https://mavlink.io/en/messages/common.html#SERIAL_CONTROL).
It can be used with normal UART ports and, on supported boards, ESC signal pins.

Typical use cases include ESC configuration tools and debugging serial peripherals over a telemetry link.

## Overview

For most users, passthrough is automatic.
When SERIAL_CONTROL traffic is sent to a supported target, PX4 starts handling that target and returns reply data over MAVLink.
You can also start and stop passthrough manually from the PX4 shell.

## Device IDs

The `device` field of `SERIAL_CONTROL` selects the target:

| Device ID | Target                  |
| --------- | ----------------------- |
| `0`       | TEL1                    |
| `1`       | TEL2                    |
| `2`       | GPS1                    |
| `3`       | GPS2                    |
| `4`       | TEL3                    |
| `5`       | TEL4                    |
| `20`      | ESC channel 0 (bitbang) |
| `21`      | ESC channel 1 (bitbang) |
| `22`      | ESC channel 2 (bitbang) |
| `23`      | ESC channel 3 (bitbang) |
| `24`      | ESC channel 4 (bitbang) |
| `25`      | ESC channel 5 (bitbang) |
| `26`      | ESC channel 6 (bitbang) |
| `27`      | ESC channel 7 (bitbang) |

The UART device path for each port (e.g. `/dev/ttyS1`) is taken from the board's
`CONFIG_BOARD_SERIAL_TEL1` / `CONFIG_BOARD_SERIAL_GPS1` / … Kconfig symbols.
If a device ID is not configured on the board the driver logs a warning and rejects the request.

## ESC Channel Mode (Bitbang UART)

Device IDs 20–27 route through a software bit-bang UART on the ESC signal pin rather than a hardware UART.
This is useful for communicating with ESCs that expose a UART telemetry or configuration port on their signal wire (e.g. BLHeli_32 passthrough, AM32, or ESC configuration tools).

Bitbang UART is implemented using a hardware timer and direct GPIO toggling.
Due to interrupt latency, reliable operation is only guaranteed up to **19200 baud**.

Because only one hardware timer is used, only one ESC channel can be active at a time.
When a request arrives for a different ESC channel the driver stops the current instance and waits up to 100 ms for it to exit before starting the new one.

Bitbang UART support requires `CONFIG_SERIALPASSTHROUGH_BITBANG=y` and the timer
is selected at build time with `CONFIG_UART_BITBANG_TIMER` (default TIM13).

## Configuration

### Build-Time

The driver is enabled via Kconfig:

```
CONFIG_DRIVERS_SERIALPASSTHROUGH=y          # Enable the driver
CONFIG_SERIALPASSTHROUGH_BITBANG=y          # Optional: ESC channel support (NuttX & STM32 only)
CONFIG_UART_BITBANG_TIMER=13                # Optional: timer instance (default TIM13)
```

### PASSTHRU_EN Parameter

When `PASSTHRU_EN` is set to `1` (and the vehicle rebooted), the normal motor output drivers
(`dshot`, `pwm_out`) are **not started** at boot.
This is required when you intend to use the ESC bitbang passthrough mode, because
the bitbang driver and the DShot/PWM driver cannot share the same ESC signal pins.

| Parameter    | Description |
| ------------ | ----------- |
| `PASSTHRU_EN` | Set to `1` to suppress normal ESC output drivers at boot. Requires reboot. |

::: warning
Setting `PASSTHRU_EN=1` disables motor control.
:::


## Bridge Application

PX4 does not yet ship a ready-made tool for using Serial Passthrough with common ESC configuration or firmware flashing tools.
To integrate it, a bridge application connects to the vehicle over MAVLink, exposes a virtual serial port (e.g. a Unix PTY) to the tool on the host, and translates traffic bidirectionally: data written to the PTY is sent as `SERIAL_CONTROL` messages with `SERIAL_CONTROL_FLAG_RESPOND | SERIAL_CONTROL_FLAG_EXCLUSIVE` set, and incoming `SERIAL_CONTROL` reply messages (with `FLAG_REPLY` set) are written back to the PTY.
To initialise the passthrough, the bridge should send one `SERIAL_CONTROL` message with the target device ID, the desired UART baud rate in the `baudrate` field, and `count=0` (no payload), then wait approximately 2 seconds for PX4 to spawn the passthrough task before sending real traffic.
For ESC bitbang mode (device IDs 20–27), the bridge must first set `PASSTHRU_EN=1` via `PARAM_SET`, confirm the `PARAM_VALUE` acknowledgement, send `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`, and wait for the FMU heartbeat to return before sending the init message — this ensures the DShot/PWM drivers are not running when the bitbang driver takes over the ESC signal pins.
We developed such a bridge internally and may upstream it in a future release.

## Limitations

- **One sender at a time:** only the MAVLink channel that sent the most recent `SERIAL_CONTROL` message receives replies.
- **ESC bitbang baud rate:** maximum reliable baud rate is 19200.
  Higher rates may work but are not guaranteed.
- **ESC channel exclusivity:** only one ESC bitbang channel can be active at a time.
- **Platform:** bitbang UART is only available on NuttX with STM32F7/H7.
  Requesting ESC bitbang on an unsupported platform returns an error.
- **Buffer size:** each instance has a 1 kB receive and 1 kB transmit buffer.
  Frames larger than 1 kB will be truncated with a warning logged.
