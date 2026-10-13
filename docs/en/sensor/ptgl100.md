# JRT PTGL-100 Laser Rangefinder

The PTGL-100 is a UART laser rangefinder with a specified measurement range of 0.2–100 m and a measurement rate of 100 Hz.
The `ptgl100` driver starts continuous measurement and publishes the standard `distance_sensor` topic used by MAVLink and ULog.
It is included in the Amovlab Flycore default firmware.

This driver targets the module configuration confirmed to use 460800 baud, 8N1, with no flow control.
The driver does not reprogram the module's baud rate.
The revised protocol specifies an unsigned 24-bit big-endian distance in millimetres, converted to metres as `distance_m = raw / 1000.0`.

::: warning
The revised manual has inconsistent command examples and baud-rate descriptions.
The driver follows the command table on printed pages 6–7, which gives `0x68` for single measurement and `0x69` for continuous measurement; the example lines still show the old `0x88`/`0x89` commands.
The specification table lists a factory baud rate of 460800, confirmed for the target module, while the protocol overview still says 115200.
Confirm the revised commands on the actual module before flight use.
Old `0x88`/`0x89` measurement replies are rejected to avoid interpreting their different distance format as millimetres.
:::

## Protocol

Frames contain eight bytes and start with `55 AA`.
The command checksum sums bytes 3–7 (one-based, excluding the header); the reply checksum sums bytes 1–7 (including the header).
Only the low eight bits of either sum are transmitted.

| Operation              | Command Bytes (Hexadecimal) |
| ---------------------- | --------------------------- |
| Single measurement     | `55 AA 68 FF FF FF FF 64`   |
| Continuous measurement | `55 AA 69 FF FF FF FF 65`   |
| Stop measurement       | `55 AA 8E FF FF FF FF 8A`   |

A measurement reply is `55 AA CMD STA D_0 D_1 D_2 SUM`.
`STA = 1` indicates success, and `(D_0 << 16) | (D_1 << 8) | D_2` is the distance in millimetres.
For example, `55 AA 69 01 01 86 A0 90` represents 100000 mm, published as 100 m.
The driver starts continuous measurement automatically; self-test (`0x80`) and stop replies are handled separately from measurements.
The baud-rate configuration command (`0x01`) from the earlier manual is absent from the revision and is not supported by this driver.

## Wiring

Connect the module to an unused flight controller UART.
UART signals use 3.3 V TTL levels.

| Module Pin | Signal | Connection                    |
| ---------- | ------ | ----------------------------- |
| 1          | GND    | Flight controller ground      |
| 2          | VCC    | Suitable 3.3–5 V supply       |
| 3          | NC     | Leave unconnected             |
| 4          | TXD    | Flight controller UART RX     |
| 5          | RXD    | Flight controller UART TX     |
| 6          | EN     | Hold low for normal operation |

Both TX and RX are required because PX4 sends the continuous measurement command.
The manual specifies that pulling EN high puts the module into low-power mode, in which it does not respond.
Check connector orientation against the module's pinout before applying power.

## Configuration

1. Ensure that the wired UART is not mapped to another service, such as MAVLink or GNSS (see [Serial Port Configuration](../peripherals/serial_configuration.md)).
2. Set `SENS_PTGL100_CFG` to that port in _QGroundControl_ and reboot.
3. If the sensor is not mounted facing down, set `SENS_PTGL100_ROT` to its mounting direction and reboot.

For example, select `TELEM 2` (`102`) for a module connected to the Flycore TELEM2 connector.
The Flycore TELEM2 connector has no VCC pin, so power the module from another source.
The driver sets 460800 baud directly; no `SER_*_BAUD` change is needed.
The default port setting is disabled, and the default orientation is downward (`25`).

Other boards can include the driver by enabling `CONFIG_DRIVERS_DISTANCE_SENSOR_PTGL100` in their board configuration.

## Viewing Data

In the _QGroundControl_ MAVLink Console, run:

```sh
ptgl100 status
listener distance_sensor -n 10
```

`status` reports the UART, baud rate, topic instance, last distance and sample age, measurement status, and error counters.
If several distance sensors are connected, select the instance reported by `status`, for example:

```sh
listener distance_sensor -i 1 -n 10
```

The MAVLink Inspector shows `DISTANCE_SENSOR`; its distance fields are in centimetres, while `distance_sensor.current_distance` in ULog and the console is in metres.
The stream rate is set by the mode of the MAVLink instance in use (`MAV_X_MODE`), as it is for other distance sensors.
The default ULog profile records the first two `distance_sensor` instances at approximately 1 Hz whenever logging is active.
No custom MAVLink message or logger topic configuration is needed for a normal single-sensor installation.
For bench recording without arming, start recording with `logger on` while the logger is running.

The driver publishes laser type, 0.2–100 m limits, and the selected orientation.
Signal quality is unknown (`-1`) for in-range measurements and invalid (`0`) for out-of-range values.
Receiver field of view is left unknown because the manual only specifies beam divergence.

## Diagnostics

The driver rejects corrupt frames and does not publish distance bytes from failed measurements.
During a communication outage it does not repeat an old distance with a new timestamp.
After one second without a complete measurement reply it retries the continuous measurement command, at most once per second.
Measurement failure replies count as communication with the module and are reported separately from communication errors.

To run the driver manually, after disabling automatic startup:

```sh
ptgl100 start -d /dev/ttyS1 -R 25
ptgl100 status
ptgl100 stop
```

The device name above corresponds to Flycore TELEM2.
When stopped, the driver attempts to send the stop measurement command before closing the UART.

## Validation

Build the firmware, then configure and run the driver-specific SITL tests with a current CMake installation:

```sh
make amovlab_flycore_default
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test -DTESTFILTER=PTGL100Parser
cmake --build build/px4_sitl_test --target px4 unit-PTGL100Parser
ctest --test-dir build/px4_sitl_test -R '^unit-PTGL100Parser$' --output-on-failure
```

Run the virtual UART integration test against the SITL test executable:

```sh
python3 src/drivers/distance_sensor/ptgl100/test_ptgl100.py --px4 build/px4_sitl_test/bin/px4
```

The integration test requires `pexpect`, `pymavlink`, and `pyulog`.
It uses a pseudo-terminal in a temporary directory and does not require a connected sensor or flight controller.

Hardware acceptance still requires checking 460800-baud communication, orientation, at least two known distances, power-cycle recovery, MAVLink output, and a recorded ULog on the actual Flycore and module.
