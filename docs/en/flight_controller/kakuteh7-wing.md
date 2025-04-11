# Holybro Kakute H7 V2

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

The [Holybro Kakute H743 Wing](https://holybro.com/products/kakute-h743-wing) is a fully featured flight controller specifically aimed at fixed-wing and VTOL applications. It has the STM32 H743 Processor running at 480 MHz and CAN Bus support, along with dual camera support & switch, ON/OFF Pit Switch, 5V, 6V/8V, 9V/12 BEC, and plug-and-play GPS, CAN, I2C ports.


::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Where to Buy

The board can be bought from one of the following shops (for example):

- [Holybro](https://holybro.com/products/kakute-h743-wing)


## Connectors and Pins

| Pin              | Function                          | PX4 default                |
| ---------------- | --------------------------------- | -------------------------- |
| GPS 1            | USART1 and I2C1                   | GPS1                       |
| R2, T2           | USART2 RX and TX                  | GPS2                       |
| R3, T3           | USART3 RX and TX                  | TELEM1                     |
| R5, T5           | USART5 RX and TX                  | TELEM2                     |
| R6, T6           | USART6 RX and TX                  | RC (PPM, SBUS, etc.) input |
| R7, T7, RTS, CTS | UART7 RX and TX with flow control | TELEM3                     |
| R8, T8           | UART8 RX and TX                   | Console                    |
| Buz-, Buz+       | Piezo buzzer                      |                            |
| M1 to M14        | Motor signal outputs              |                            |

<a id="bootloader"></a>

## PX4 Bootloader Update

The board comes pre-installed with [Betaflight](https://github.com/betaflight/betaflight/wiki).
Before the PX4 firmware can be installed, the _PX4 bootloader_ must be flashed.
Download the [holybro_kakuteh7-wing.hex](https://github.com/PX4/PX4-Autopilot/raw/main/docs/assets/flight_controller/kakuteh7-wing/holybro_kakuteh7-wing_bootloader.hex) bootloader binary and read [this page](../advanced_config/bootloader_update_from_betaflight.md) for flashing instructions.

## Building Firmware

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make holybro_kakuteh7-wing_default
```

## Installing PX4 Firmware

::: info
KakuteH7-wing is supported with PX4 master & PX4 v1.16 or newer..
Prior to that release you will need to manually build and install the firmware.
:::

Firmware can be manually installed in any of the normal ways:

- Build and upload the source:

  ```
  make holybro_kakuteh7-wing_default upload
  ```

- [Load the firmware](../config/firmware.md) using _QGroundControl_.
  You can use either pre-built firmware or your own custom firmware.

## Serial Port Mapping

| UART   | Device     | Port                  | Default function |
| ------ | ---------- | --------------------- | ---------------- |
| USART1 | /dev/ttyS0 | GPS 1                 | GPS1             |
| USART2 | /dev/ttyS1 | R2, T2                | GPS2             |
| USART3 | /dev/ttyS2 | R3, T3                | TELEM1           |
| UART5  | /dev/ttyS3 | R5, T5                | TELEM2           |
| USART6 | /dev/ttyS4 | R6, (T6)              | RC input         |
| UART7  | /dev/ttyS5 | R7, T7, RTS, CTS      | TELEM3           |
| UART8  | /dev/ttyS6 | R8, T8                | Console          |

## Debug Port

### System Console

UART8 RX and TX are configured for use as the [System Console](../debug/system_console.md).
