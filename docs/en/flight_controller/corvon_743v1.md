# CORVON 743V1

<Badge type="tip" text="PX4 v1.18" />

:::warning
PX4 does not manufacture this (or any) autopilot. Contact the manufacturer for hardware support or compliance issues.
:::

The _CORVON 743v1_ is a flight controller designed by Feikong Technology Co., Ltd under the CORVON brand.
It features a powerful STM32H743 processor, multiple high-performance IMUs, and an extensive array of interfaces.

The board uses [Pixhawk Autopilot Standard Connections](https://docs.px4.io/main/en/flight_controller/autopilot_pixhawk_standard.html).

<img src="../../assets/flight_controller/corvon_743v1/corvon_743v1_top.jpg" width="400px" title="CORVON 743v1 Top Baseboard" /> <img src="../../assets/flight_controller/corvon_743v1/corvon_743v1_bottom.jpg" width="400px" title="CORVON 743v1 Bottom Interfaces" />

## Key Features

- **MCU:** STM32H743 MCU (32 Bit Arm® Cortex®-M7, 480MHz, 2MB Flash, 1MB RAM)
- **IMU:** Bosch BMI088, BMI270
- **Barometer:** DPS310
- **Interfaces:**
  - 6x UARTs
  - 1x CAN (UAVCAN)
  - Dedicated RC Input
  - I2C & SPI
- **Power:** ADC for battery voltage monitoring

## Where to Buy

Order from [CORVON](https://corvon.tech).

## Specifications

### Processors & Sensors

- **FMU Processor:** STM32H743
  - 32 Bit Arm® Cortex®-M7, 480MHz
  - 2MB Flash, 1MB RAM
- **On-board Sensors:**
  - Accel/Gyro: Bosch BMI088, BMI270
  - Barometer: DPS310

### Interfaces

- 6 Serial Ports (UARTs)
- 1 CAN Bus (UAVCAN)
- Dedicated RC Input
- PWM outputs (DShot supported)

## Building Firmware

::: tip
Most users will not need to build this firmware (from PX4 v1.18).
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make corvon_743v1_default
```

## Serial Port Mapping

| UART   | Device     | Port   |
| ------ | ---------- | ------ |
| UART4  | /dev/ttyS0 | TELEM1 |
| USART2 | /dev/ttyS1 | TELEM2 |
| USART1 | /dev/ttyS2 | GPS1   |
| USART3 | /dev/ttyS3 | TELEM3 |
| USART6 | /dev/ttyS4 | RC     |
| UART8  | /dev/ttyS5 | URT6   |
| UART7  | /dev/ttyS6 | TELEM4 |
