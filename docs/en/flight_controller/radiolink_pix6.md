# RadiolinkPIX6 Flight Controller

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://radiolink.com.cn/) for hardware support or compliance issues.
:::

The autopilot is recommended for commercial systems integration, but is also suitable for academic research and any other use.

![RadiolinkPIX6](http://www.radiolink.com.cn/firmware/wiki/RadiolinkPIX6/RadiolinkPIX6.png)

The Radiolink PIX6 is a high-performance flight controller. Featuring STM32F7 cpu, vibration isolation of IMUs, redundant IMUs,  integrated OSD chip,  IMU heating, and DShot.

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Quick Summary

- Processor
  - 32-bit ARM Cortex M7 core with DPFPU - STM32F765VIT6
  - 216 MHz/512 KB RAM/2 MB Flash
  - 32-bit IOMCU co-processor - STM32F100
  - 32KB FRAM - FM25V02A
  - AT7456E OSD
- Sensors
  - Bosh BMI088 IMU (accel, gyro)
  - InvenSense ICM-42688 IMU (accel, gyro)
  - SPA06 barometer
  - IST8310 magnetometer
- Power
  - SMBUS/I2C Power Module Inputs(I2C)
  - voltage and current monitor inputs(Analog)
- Interfaces
  - 16 PWM Outputs with independent power rail for external power source
  - 5x UART serial ports, 2 with HW flow control
  - Camera Input and Video Output
  - PPM/SBUS input, DSM/SBUS input
  - RSSI (PWM or voltage) input
  - I2C, SPI, 2x CAN, USB
  - 3.3V and 6.6V ADC inputs
  - Buzzer and Safety Switch
  - microSD card
- Weight and Dimensions:
  - Weight 80g
  - Size 94mm x 51.5mm x 14.5mm

## Where to Buy
[Radiolink Amazon](https://www.radiolink.com.cn/pix6_where_to_buy)（International users）

[Radiolink Taobao](https://item.taobao.com/item.htm?spm=a21dvs.23580594.0.0.1d292c1bNMdSqV&ft=t&id=815993357068&skuId=5515756705284)（China Mainland user）


## Connector assignments

### Top View

 <img src="http://www.radiolink.com.cn/firmware/wiki/RadiolinkPIX6/Top_View.png" alt="Top_View" style="zoom: 50%;" />

### Left View

 <img src="http://www.radiolink.com.cn/firmware/wiki/RadiolinkPIX6/Left_View.png" alt="Right_View" style="zoom: 67%;" />

 ### Right View

 <img src="http://www.radiolink.com.cn/firmware/wiki/RadiolinkPIX6/Right_View.png" alt="Left_View" style="zoom: 67%;" />

### Rear View

 <img src="http://www.radiolink.com.cn/firmware/wiki/RadiolinkPIX6/Rear_View.png" alt="Rear" style="zoom: 50%;" />

## Pinouts

### TELEM1, TELEM2 ports

| Pin  | Signal  | Volt  |
| ---- | ------- | ----- |
| <span style="display:inline-block;width:30px"> 1 </span> | <span style="display:inline-block;width:120px"> VCC </span> | <span style="display:inline-block;width:600px"> +5V </span> |
| 2    | TX(OUT) | +3.3V |
| 3    | RX(IN)  | +3.3V |
| 4    | CTS     | +3.3V |
| 5    | RTS     | +3.3V |
| 6    | GND     | GND   |

### OSD

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | GND                                                          | GND                                                          |
| 2                                                          | VOUT                                                         | +3.3V                                                        |
| 3                                                          | VCC                                                          | +5V                                                          |
| 4                                                          | GND                                                          | GND                                                          |
| 5                                                          | VCC                                                          | +5V                                                          |
| 6                                                          | VIN                                                          | +3.3V                                                        |

### I2C port

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | SCL                                                          | +3.3V (pullups)                                              |
| 3                                                          | SDA                                                          | +3.3V (pullups)                                              |
| 4                                                          | GND                                                          | GND                                                          |

### CAN1, CAN2 ports

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | CAN_H                                                        | +12V                                                         |
| 3                                                          | CAN_L                                                        | +12V                                                         |
| 4                                                          | GND                                                          | GND                                                          |

### GPS1 port

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | TX(OUT)                                                      | +3.3V                                                        |
| 3                                                          | RX(IN)                                                       | +3.3V                                                        |
| 4                                                          | SCL                                                          | +3.3V                                                        |
| 5                                                          | SDA                                                          | +3.3V                                                        |
| 6                                                          | GND                                                          | GND                                                          |

### GPS2 Port

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | TX(OUT)                                                      | +3.3V                                                        |
| 3                                                          | RX(IN)                                                       | +3.3V                                                        |
| 4                                                          | SCL                                                          | +3.3V                                                        |
| 5                                                          | SDA                                                          | +3.3V                                                        |
| 6                                                          | GND                                                          | GND                                                          |

### SPI

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | SPI_SCK                                                      | +3.3V                                                        |
| 3                                                          | SPI_MISO                                                     | +3.3V                                                        |
| 4                                                          | SPI_MOSI                                                     | +3.3V                                                        |
| 5                                                          | !SPI_NSS1                                                    | +3.3V                                                        |
| 6                                                          | !SPI_NSS2                                                    | +3.3V                                                        |
| 7                                                          | DRDY                                                         | +3.3V                                                        |
| 8                                                          | GND                                                          | GND                                                          |

### POWER1

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | VCC                                                          | +5V                                                          |
| 3                                                          | CURRENT                                                      | up to +3.3V                                                  |
| 4                                                          | VOLTAGE                                                      | up to +3.3V                                                  |
| 5                                                          | GND                                                          | GND                                                          |
| 6                                                          | GND                                                          | GND                                                          |

### POWER2

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | VCC                                                          | +5V                                                          |
| 3                                                          | SCL                                                          | +3.3V                                                        |
| 4                                                          | SDA                                                          | +3.3V                                                        |
| 5                                                          | GND                                                          | GND                                                          |
| 6                                                          | GND                                                          | GND                                                          |

### ADC 3.3V

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | ADC IN1                                                      | up to +3.3V                                                  |
| 3                                                          | GND                                                          | GND                                                          |
| 4                                                          | ADC IN2                                                      | up to +3.3v                                                  |
| 5                                                          | GND                                                          | GND                                                          |

### ADC 6.6V

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | ADC IN                                                       | up to 6.6V                                                   |
| 3                                                          | GND                                                          | GND                                                          |

### USB remote port

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | USB VDD                                                      | +5V                                                          |
| 2                                                          | DM                                                           | +3.3V                                                        |
| 3                                                          | DP                                                           | +3.3V                                                        |
| 4                                                          | GND                                                          | GND                                                          |

### SWITCH

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +3.3V                                                        |
| 2                                                          | !IO_LED_SAFETY                                               | GND                                                          |
| 3                                                          | SAFETY                                                       | GND                                                          |

### Buzzer port

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | BUZZER-                                                      | +5V                                                          |

### Spektrum/DSM Port

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +3.3V                                                        |
| 2                                                          | GND                                                          | GND                                                          |
| 3                                                          | Signal                                                       | +3.3V                                                        |

### Debug port

| <span style="display:inline-block;width:30px"> Pin </span> | <span style="display:inline-block;width:120px"> Signal </span> | <span style="display:inline-block;width:600px"> Volt </span> |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1                                                          | VCC                                                          | +5V                                                          |
| 2                                                          | FMU_SWCLK                                                    | +3.3V                                                        |
| 3                                                          | FMU_SWDIO                                                    | +3.3V                                                        |
| 4                                                          | TX(UART7)                                                    | +3.3V                                                        |
| 5                                                          | RX(UART7)                                                    | +3.3V                                                        |
| 6                                                          | IO_SWCLK                                                     | +3.3V                                                        |
| 7                                                          | IO_SWDIO                                                     | +3.3V                                                        |
| 8                                                          | GND                                                          | GND

## Building Firmware

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make radiolink_PIX6_default
```

## Installing PX4 Firmware

The firmware can be installed in any of the normal ways:

- Build and upload the source

  ```sh
  make radiolink_PIX6_default upload
  ```

- [Load the firmware](../config/firmware.md) using _QGroundControl_.
  You can use either pre-built firmware or your own custom firmware.

## PX4 Configuration

In addition to the [basic configuration](../config/index.md), the following parameters are important:

| Parameter                                                            | Setting                                                                                                                 |
| -------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG) | This should be disabled since the board does not have an internal mag. You can enable it if you attach an external mag. |

## Serial Port Mapping

| UART   | Device     | Port                                  |
| ------ | ---------- | ------------------------------------- |
| UART1  | /dev/ttyS0 | GPS1                                  |
| USART2 | /dev/ttyS1 | TELEM1 (flow control)                 |
| USART3 | /dev/ttyS2 | TELEM2 (flow control)                 |
| UART4  | /dev/ttyS3 | GPS2                                  |
| UART7  | /dev/ttyS4 | Debug Console                         |
| UART8  | /dev/ttyS5 | PX4IO                                 |

## Analog inputs

The RadiolinkPIX6 has 3 analog inputs, one 6V tolerant and two 3.3V tolerant

 - ADC Pin12 -> ADC 6.6V Sense
 - ADC Pin4   -> ADC IN1 3.3V Sense
 - ADC Pin13 -> ADC IN2 3.3V Sense

## Connectors

Unless noted otherwise all connectors are JST GH
