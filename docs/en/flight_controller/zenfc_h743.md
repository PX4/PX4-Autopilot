# ZenFC H743

<!-- TODO: set to the PX4 release this target first merges into -->
<Badge type="tip" text="PX4 v1.18" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.zenithratech.com/contact) for hardware support or compliance issues.
:::

The _ZenFC H743_ is a flight controller designed by Zenithra Tech.
It is built around the STM32H743 processor with dual Bosch BMI088 IMUs for sensor redundancy, an onboard barometer, and an integrated OSD.


<p align="center">
  <img src="../../assets/flight_controller/zenfc_h743/zenfc_h743.png" alt="ZenFC H743" width="600"/>
</p>


::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Key Features

- **MCU:**
  - STM32H743VIT6 (32-bit Arm® Cortex®-M7, 480 MHz)
- **IMU:**
  - 2x Bosch BMI088 (accel + gyro, independent SPI buses for redundancy)
- **Barometer:**
  - Bosch BMP388
- **OSD:**
  - Onboard AT7456E OSD chip
- **Interfaces:**
  - 7x UARTs
  - 1x CAN
  - 2x external I2C (I2C1 with GPS connector and dedicated I2C2 connector)
  - 8x PWM outputs (Dshot supported)
  - microSD card slot
  - 1x USB Type-C
- **Power:**
  - 2x ADC (V_BAT & current sense)
  - Battery Input range - 2S-6S
  - BEC Outputs
    - 9V 3A cont.
    - 5V 3A cont.

## Dimensions


- **Mounting:** 30.5 mm x 30.5 mm /Φ4mm hole
- **Dimensions:** 35mm x 35mm x 8mm
- **Weight:** 9g


<p align="center">
  <img src="../../assets/flight_controller/zenfc_h743/zenfc_h743_dimensions.png" alt="ZenFC H743 Dimensions" width="700"/>
</p>


## Where to Buy

Order from [Zenithra Tech](https://www.zenithratech.com/solutions/components).

## Flight Controller Layout

<p align="center">
  <img src="../../assets/flight_controller/zenfc_h743/zenfc_h743_layout_front.png" alt="ZenFC H743 Layout Front" width="450"/>
</p>

<p align="center">
  <img src="../../assets/flight_controller/zenfc_h743/zenfc_h743_layout_back.png" alt="ZenFC H743 Layout Back" width="450"/>
</p>

## Serial Port Mapping

| UART   | Device     | PX4 Port |
| ------ | ---------- | -------- |
| USART1 | /dev/ttyS0 | GPS1     |
| USART2 | /dev/ttyS1 | TEL3     |
| USART3 | /dev/ttyS2 | TEL4     |
| UART5  | /dev/ttyS3 | TEL1     |
| USART6 | /dev/ttyS4 | RC       |
| UART7  | /dev/ttyS5 | TEL2     |
| UART8  | /dev/ttyS6 | URT6     |


## Connectors & Pinout

Board uses JST SH connectors for all interfaces.

<p align="center">
  <img src="../../assets/flight_controller/zenfc_h743/zenfc_h743_pinouts_front.png" alt="ZenFC H743 Pinouts Front" width="600"/>
</p>

<p align="center">
  <img src="../../assets/flight_controller/zenfc_h743/zenfc_h743_pinouts_back.png" alt="ZenFC H743 Pinouts Back" width="600"/>
</p>

**Note** - SBus pin on DJI O3/O4 port is internally connected to RX6 (RCIN).

### PWM Outputs

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1
- Outputs 5-6 in group2
- Outputs 7-8 in group3

Note: All outputs support PWM and Dshot protocol.

## PX4 Configuration

In addition to the [basic configuration](../config/index.md), the following
parameter is important:

| Parameter                                                            | Setting                                                                                                  |
| --------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG) | Disabled by default since the board has no internal magnetometer. Enable it if you attach an external mag. |

## Building Firmware

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make zenfc_h743_default
```

## Installing PX4 Firmware

The firmware can be installed in any of the normal ways:

- Build and upload the source:

  ```sh
  make zenfc_h743_default upload
  ```

- [Load the firmware](../config/firmware.md) using _QGroundControl_.
  You can use either pre-built firmware or your own custom firmware.

## Further info

- [Zenithra Tech.](https://www.zenithratech.com/contact)
