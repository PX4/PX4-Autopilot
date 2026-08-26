# u-blox Diagnostics with u-center

<Badge type="tip" text="main (PX4 v2.0)" />

[u-center](https://www.u-blox.com/en/product/u-center) is u-blox's desktop tool for monitoring a receiver's live satellite, signal, and position data, and for reading and writing its configuration.

Setting [GPS_UBX_MODE](#GPS_UBX_MODE) to `7` configures a u-blox receiver's `UART2` as a UBX diagnostic port for [u-center](https://www.u-blox.com/en/product/u-center), while the receiver continues to serve the autopilot on `UART1` (as usual).
This is useful for diagnosing poor fixes, interference, or RTK issues without disconnecting the autopilot.

:::info
Heading and static-base-on-UART2 setups cannot use this feature.
For more information see [Constraints](#constraints) below.
:::

## Підключення

Connect the adapter's RX to the receiver's `UART2` TX, its TX to `UART2` RX, and share ground.
u-blox UART pins are 3.3 V.

## Конфігурація PX4

The parameters to set:

| Parameter                                                                                                                                 | Налаштування                                                                          |
| ----------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| <a id="GPS_UBX_MODE"></a>[GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) | `u-center on UART2` (7)                                            |
| [GPS_UBX_BAUD2](../advanced_config/parameter_reference.md#GPS_UBX_BAUD2)                        | UART2 baudrate, must match the USB-serial adapter (default 230400) |

Both parameters take effect after a reboot.

## Messages Enabled on UART2

The following messages are enabled when `GPS_UBX_MODE` is set to `7`:

| Повідомлення                                 | Rate                    |
| -------------------------------------------- | ----------------------- |
| `NAV-PVT`, `NAV-DOP`, `NAV-STATUS`, `MON-RF` | Every navigation epoch  |
| `NAV-HPPOSLLH`                               | Every epoch, except M9  |
| `RXM-RTCM`                                   | Every epoch, M9 and F9P |
| `NAV-SAT`                                    | Every 10th epoch        |

The set runs around 300 bytes per navigation epoch (GPS update cycle), roughly triple that on the epoch carrying `NAV-SAT`.

The default `UART2` baud rate (230400) covers the 25 Hz maximum; 115200 is the practical floor at the 10 Hz default rate.
Below that, the `NAV-SAT` epoch saturates the link, and a u-blox receiver drops messages on a congested port rather than throttling: the result is gaps in u-center.

## Обмеження

- UBX input is enabled on `UART2`, so anything connected there can reconfigure the receiver.
  Use this on the bench, not in flight.
- The driver writes to the receiver's RAM configuration layer on every boot.
  Changes made from u-center to the RAM layer are lost at the next power cycle; changes written to BBR or flash persist and will fight the driver.
- `GPS_UBX_MODE` 1, 2 and 5 use `UART2` for RTCM, so heading and static-base-on-UART2 setups cannot use mode 7.
- The mode is skipped with a warning on M10 and F10 receivers, and on receivers older than u-blox protocol version 27.
