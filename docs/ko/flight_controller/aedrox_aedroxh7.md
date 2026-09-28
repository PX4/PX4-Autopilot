# AEDROX AEDROXH7

<Badge type="tip" text="PX4 v1.18" />

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.aedrox.com/) for hardware support.
:::

The AEDROXH7 is an STM32H743-based FPV / racing flight controller from AEDROX.

For full hardware documentation and pinouts, see the [manufacturer documentation](https://aedrox.gitbook.io/docs).

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## 주요 특징

- MCU: STM32H743VIM6 32-bit processor running at 480 MHz
- IMU: ICM-42688-P
- Barometer: DPS310 (some revisions ship DPS368, supported by the same driver)
- 128 MB W25N NAND flash for logging (mounted as littlefs at `/fs/flash`)
- MAX7456 analog OSD
- 6 UARTs (TEL1, GPS1, RC, TEL2, ESC telemetry, DJI/MSP HD OSD / debug console)
- 1 internal I2C bus (barometer), 1 external I2C bus (user connector)
- 1 CAN bus (CAN1) with silent-pin control
- 8 PWM motor outputs (Bidirectional DShot capable)
- 1 addressable LED strip pad, 2 general GPIO
- Battery voltage and current monitoring
- USB Type-C

No on-board magnetometer; PX4 runs with `SYS_HAS_MAG=0` and EKF2 gravity fusion enabled by default.

## Where to Buy {#store}

- [aedrox.com](https://www.aedrox.com/).

## 커넥터 및 핀

### UART

| 포트        | MCU peripheral | 장치         | 기능                                              |
| --------- | -------------- | ---------- | ----------------------------------------------- |
| `SERIAL1` | USART1         | /dev/ttyS0 | TELEM1 (MAVLink)             |
| `SERIAL2` | USART2         | /dev/ttyS1 | GPS1                                            |
| `SERIAL3` | USART3         | /dev/ttyS2 | RC 입력                                           |
| `SERIAL4` | UART4          | /dev/ttyS3 | TELEM2                                          |
| `SERIAL7` | UART7          | /dev/ttyS4 | ESC telemetry (RX only)      |
| `SERIAL8` | UART8          | /dev/ttyS5 | Debug shell (system console) |

`SERIAL7` is wired RX-only by the vendor (intended for ESC telemetry).
`SERIAL8` is the system console at 57600 8N1 (device: `/dev/ttyS5`).

:::info
The vendor brings `SERIAL8` out on the HD VTX connector (intended for DJI / MSP DisplayPort), but the default PX4 config uses it as the system console. To use it for MSP DisplayPort instead, move the console off UART8, then start the [msp_osd](../modules/modules_driver.md#msp-osd) driver on `/dev/ttyS5`.
:::

### PWM Outputs {#pwm_outputs}

8 PWM motor outputs (M1-M8), all [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) (RPM telemetry) capable.

The M5-M8 motor connector additionally exposes two user-controllable GPIOs (`GPIO1` / `GPIO2`, on PA2 / PA3) alongside the motor signals.
Toggle them at runtime with the `gpio` command (see "User GPIOs" below).

A separate addressable-LED-strip pad is brought out; currently driven as a plain GPIO in this port .

### Status LEDs

| Silkscreen | MCU pin | 색상 | 기능                                                                                              |
| ---------- | ------- | -- | ----------------------------------------------------------------------------------------------- |
| LED1       | PE2     | 청색 | MCU activity, solid when armed                                                                  |
| LED2       | -       | 녹색 | IMU power-supply indicator                                                                      |
| LED3       | -       | 녹색 | MCU / baro / OSD power-supply indicator                                                         |
| LED4       | PE5     | 녹색 | MCU activity, blinks based on flight state (preflight / disarmed / failsafe) |
| LED5       | -       | -  | 10v VTX rail indicator (lit when VTX power is enabled)                       |

### Other I/O

| 기능              | MCU pin   | 참고                                                                                                                                                       |
| --------------- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 부저              | PA7       | Drives an NPN low-side switch, use a basic 2-pin active buzzer (e.g. TMB12A05)                        |
| VTX power       | PB1       | Active high                                                                                                                                              |
| Camera switch   | PD15      | Active high, currently hard-coded to camera 1 at boot (no runtime control yet); use `gpio write D15 1` from nsh to switch to camera 2 |
| PINIO 1 / 2     | PA2 / PA3 | User-controllable GPIOs via `gpio` command                                                                                                               |
| CAN1 silent     | PD12      | Driven low to enable CAN bus                                                                                                                             |
| Battery V sense | PC0       | ADC1 IN10, calibrate `BAT1_V_DIV` per build                                                                                                              |
| Battery I sense | PC1       | ADC1 IN11, calibrate `BAT1_A_PER_V` per ESC, or use DShot telemetry for current (`BAT1_SOURCE`)                                       |
| USB VBUS sense  | PC15      | Non-standard pin for VBUS sense                                                                                                                          |

### Analog OSD (MAX7456)

The MAX7456 driver is built in but disabled by default. To enable, set [`OSD_ATXXXX_CFG`](../advanced_config/parameter_reference.md#OSD_ATXXXX_CFG) to `1` (NTSC) or `2` (PAL) and reboot. The `atxxxx` driver auto-starts and overlays PX4 status on top of the camera video.

## 부트로더 업데이트

Before PX4 firmware can be installed, the PX4 bootloader must be flashed.
Download the [aedrox_aedroxh7_bootloader.bin](https://github.com/PX4/PX4-Autopilot/blob/main/boards/aedrox/aedroxh7/extras/aedrox_aedroxh7_bootloader.bin) bootloader binary and follow the [bootloader update from Betaflight / DFU](../advanced_config/bootloader_update_from_betaflight.md) flashing instructions.

## 펌웨어 빌드

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make aedrox_aedroxh7_default
```

## 펌웨어 설치

Firmware can be installed in any of the normal ways:

- Build and upload the source:

  ```sh
  make aedrox_aedroxh7_default upload
  ```

- [Load the firmware](../config/firmware.md) using _QGroundControl_.
  미리 빌드된 펌웨어나 사용자 지정 펌웨어를 사용할 수 있습니다.

### Flash Storage Troubleshooting

The AEDROXH7 uses a 128 MB NAND flash (W25N) with a littlefs filesystem for [logging](../dev_log/logging.md).
If the flash filesystem becomes corrupted, you can reformat it from the [System Console](../debug/system_console.md):

```sh
mklittlefs /dev/mtd0 /fs/flash
```

This will erase all data on the flash and create a fresh littlefs filesystem.
The filesystem is immediately available after the command completes.

### 시스템 콘솔

UART8 is configured for use as the [System Console](../debug/system_console.md) at 57600 8N1.
