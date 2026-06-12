# Serial Passthrough (MAVLink SERIAL_CONTROL)

Serial Passthrough is a driver that bridges a MAVLink link to a hardware UART or an ESC signal pin,
using the MAVLink [SERIAL_CONTROL](https://mavlink.io/en/messages/common.html#SERIAL_CONTROL) message.
It allows a GCS, companion computer, or any MAVLink-speaking client to transparently read from and write to a serial device on the flight controller without any additional infrastructure.

## How It Works

The driver maintains two thread-safe byte buffers and one dedicated OS task per device.

Data flow:

```
GCS ──SERIAL_CONTROL──► MavlinkReceiver ──pushFromMavlink()──► [rx buffer] ──► UART / ESC pin
GCS ◄─SERIAL_CONTROL(REPLY)─ Mavlink loop ◄─popToMavlink()─── [tx buffer] ◄── UART / ESC pin
```

1. When a `SERIAL_CONTROL` message arrives on any MAVLink channel the receiver
   calls `SerialPassthrough::startForDevice()` to lazily start a task for that
   device if none is running yet, then pushes the payload bytes into the
   driver's receive buffer.
2. The driver task drains the receive buffer to the UART (or ESC pin) and
   reads any reply bytes back into its transmit buffer.
3. The MAVLink main loop drains the transmit buffer and sends back a
   `SERIAL_CONTROL` message with the `FLAG_REPLY` flag set, directed at the
   original sender.

Up to 8 passthrough instances can run simultaneously, one per device.
Only a single sender per instance is supported at a time.
Simultaneous `SERIAL_CONTROL` messages from different senders produce undefined behaviour.

## Device IDs

The `device` field of `SERIAL_CONTROL` selects the target:

| Device ID | Target |
| --------- | ------ |
| `0`       | TEL1   |
| `1`       | TEL2   |
| `2`       | GPS1   |
| `3`       | GPS2   |
| `4`       | TEL3   |
| `5`       | TEL4   |
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

## Usage

The driver starts automatically on the first `SERIAL_CONTROL` message received for a given device.
Manual control is available via the PX4 shell:

```sh
# Start manually on a UART at 115200 baud
serialpassthrough start -d /dev/ttyS1 -b 115200

# Start on ESC channel 0 via bitbang at 19200 baud
serialpassthrough start -e 0 -b 19200

# Show all running instances
serialpassthrough status

# Stop all instances
serialpassthrough stop
```

### Options

| Flag | Argument | Description |
| ---- | -------- | ----------- |
| `-d` | `<path>` | Serial device path (e.g. `/dev/ttyS1`) |
| `-b` | `<baud>` | Baud rate (default 115200) |
| `-e` | `0`–`7`  | ESC bitbang channel, instead of `-d` |
| `-x` |          | Swap RX/TX pins |
| `-s` |          | Single-wire (half-duplex) mode |

## Limitations

- **One sender at a time:** only the MAVLink channel that sent the most recent `SERIAL_CONTROL` message receives replies.
- **ESC bitbang baud rate:** maximum reliable baud rate is 19200.
  Higher rates may work but are not guaranteed.
- **ESC channel exclusivity:** only one ESC bitbang channel can be active at a time.
- **Platform:** bitbang UART is only available on NuttX with STM32F7/H7.
  Requesting ESC bitbang on an unsupported platform returns an error.
- **Buffer size:** each instance has a 1 kB receive and 1 kB transmit buffer.
  Frames larger than 1 kB will be truncated with a warning logged.
