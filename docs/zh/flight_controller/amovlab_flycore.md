# Amovlab Flycore

::: warning
PX4 不生产该（或任何）自驾仪。
硬件支持与合规问题请联系 [厂商](https://amovlab.com/)。
:::

::: info
本飞控计划纳入 [厂商支持](../flight_controller/autopilot_manufacturer_supported.md) 板卡列表。
:::

## 传感器与总线说明

默认 Flycore PX4 移植与出厂硬件一致：

- **板载传感器：** BMI088 + ICM42688P + MS5611（**无磁力计**）。
- **外置 I2C1 / I2C4：** 可连接用户自购的 **可选** GPS/罗盘一体模块或其他 I2C 外设。
- **默认固件：** `rc.board_sensors` 仅启动板载 IMU 与气压计；**不会** 对默认未安装的磁力计或其他外置传感器执行 `start`。

默认构建中启用 `CONFIG_COMMON_MAGNETOMETER`，以便用户在加装兼容的外置罗盘后，可在 [系统控制台](../debug/system_console.md) 中手动启动对应驱动（建议先用 `i2cdetect` 确认总线与地址）。

无外置磁力计时，飞行偏航主要依赖 GPS 航向（在可用时）及 QGroundControl 中的估计器配置；无 GPS 环境请合理规划任务。

## 概述

- **MCU：** STM32H743（Cortex-M7）
- **板载 IMU：** Bosch BMI088（SPI1）、InvenSense ICM42688P（SPI2）
- **板载气压计：** MS5611（SPI1）
- **PWM：** 10 路 FMU 输出（无 PX4IO 协处理器）
- **CAN：** 双路外置 CAN（FDCAN1 / FDCAN2）
- **USB：** 产品字符串 `AMOVLAB FLYCORE`

## 串口映射

| 接口   | 设备节点       |
| ------ | -------------- |
| TELEM1 | `/dev/ttyS0`   |
| TELEM2 | `/dev/ttyS1`   |
| GPS1   | `/dev/ttyS2`   |
| TELEM3 | `/dev/ttyS3`   |
| RC     | `/dev/ttyS4`   |
| GPS2   | `/dev/ttyS5`   |

## 编译固件

```sh
make amovlab_flycore_default
```

Bootloader：

```sh
make amovlab_flycore_bootloader
```

烧录（USB）：

```sh
make amovlab_flycore_default upload
```

## Bootloader / 板 ID

- **board_id：** 106（`boards/amovlab/flycore/firmware.prototype`）
- Bootloader USB 产品名：`PX4 BL AMOV FLYCORE`

## 默认机架

硬件类型 `FLYCORE0000` 时，`rc.board_defaults` 将 `SYS_AUTOSTART` 设为 **4014**（通用多旋翼机架类）。
请在 QGroundControl 中按实际机型调整机架。

## 可选外置磁力计

若在 I2C1 或 I2C4 上安装 GPS/罗盘模块，请先探测总线（`i2cdetect -b 1` / `i2cdetect -b 4`），再手动启动驱动，例如：

```sh
ist8310 -X -b 1 -R 10 start
```

`-R` 旋转参数须根据模块安装方向与手册设置。

## 维护

- **厂商：** Amovlab
- **PX4 目标：** `amovlab_flycore_default`
