# Allocor Atlas

<Badge type="tip" text="main (PX4 v2.0)" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.allocor.tech/atlas-product-page) for hardware support or compliance issues.
:::

Atlas is a flight controller manufactured by Allocor, based on the [FMUv6X reference design](https://github.com/pixhawk/Pixhawk-Standards).

It uses an STM32H753II FMU processor paired with a PX4IO v2 coprocessor for PWM output and safety-switch handling.

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Specifications {#specifications}

### Processor {#processor}

- STM32H753II FMU (Arm® Cortex®-M7, 480MHz, 2MB flash, 1MB RAM)
- PX4IO v2 coprocessor

### Sensors {#sensors}

- [Bosch BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/) accelerometer/gyroscope (SPI3)
- [InvenSense ICM-42688-P](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/) IMU (SPI2)
- [InvenSense IIM-42652](https://invensense.tdk.com/products/motion-tracking/6-axis/iim-42652/) IMU (SPI4)
- [Bosch BMP388](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/) barometer (I2C)
- [Bosch BMM350](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm350/) magnetometer (I2C, bus 4, address `0x14`)

All three IMUs are on separate SPI buses for sensor redundancy.

### Interfaces {#interfaces}

<!-- placeholder
PWM outputs (8 FMU + 8 PX4IO), serial ports with labels, I2C (I2C4 internal), SPI, CAN, Ethernet, USB, RC input, parameter storage (FM25V02A FRAM), storage, as `- **Label:** value` items.
Good examples: ark_v6xrt.md#interfaces, cuav_pixhawk_v6x.md#interfaces
-->

### Electrical data {#electrical_data}

<!-- placeholder
Input voltage per power input, current draw (with the heater's share), power monitoring and which monitor is on by default.
Good examples: ark_v6xrt.md#electrical-data, pixhawk6x.md#electrical-data
-->

### Mechanical data {#mechanical_data}

<!-- placeholder
Dimensions, weight, operating temperature.
Good examples: ark_v6xrt.md#mechanical-data, pixhawk6x.md#mechanical-data
-->

## Where to Buy {#store}

<!-- placeholder
Link to where the board can be bought, or to Allocor's contact page if it's sold on request.
Good examples: ark_v6xrt.md#store, cuav_pixhawk_v6x.md#store
-->

## Pinouts {#pinouts}

<!-- placeholder
A table per connector (P1, P2, P3): pin, signal, voltage. Or a link to a public (non-draft) manufacturer pinout document.
Good examples: amovlab_flycore.md#pinouts, radiolink_pix6.md#pinouts, cuav_pixhawk_v6x.md#pinouts
-->

## Power {#power}

- 2 digital power bricks (voltage/current sensed over I2C via INA226/INA228/INA238)

## Voltage Ratings {#voltage_ratings}

<!-- placeholder
Normal operation maximum ratings and absolute maximum ratings for VIN-A, VIN-B, USB and the servo rail, in the order the board draws power.
Good examples: cuav_pixhawk_v6x.md#voltage-ratings, agam_v6xrt.md#voltage-ratings
-->

## PWM Outputs {#pwm_outputs}

<!-- placeholder
16 outputs: 8 IO (MAIN) and 8 FMU (AUX). DShot and bidirectional DShot on AUX 1-6, not 7-8. Groups: AUX 1-4 (Timer5), 5-6 (Timer4), 7-8 (Timer12). Same protocol and rate within a group.
Good examples: nwblue_pro-h757.md#pwm-outputs, amovlab_flycore.md#pwm_outputs, ark_v6xrt.md#pwm_outputs
-->

## Telemetry Radios (Optional) {#telemetry}

<!-- placeholder
Which TELEM ports a telemetry radio can use, noting that TELEM2 is configured for the mission computer by default.
Good examples: amovlab_flycore.md#telemetry
-->

## Ethernet {#ethernet}

<!-- placeholder
Speed, the onboard switch and ports, the default MAVLink (MAV_2_CONFIG 1000) and uXRCE-DDS (agent 10.41.10.1) configuration, and a link to ../advanced_config/ethernet_setup.md.
Good examples: siyi-unifc-6-pico.md#ethernet
-->

## Serial Port Mapping {#serial_port_mapping}

| UART   | Device     | Port          |
| ------ | ---------- | ------------- |
| USART1 | /dev/ttyS0 | GPS1          |
| USART2 | /dev/ttyS1 | TELEM3        |
| USART3 | /dev/ttyS2 | Debug Console |
| UART4  | /dev/ttyS3 | TELEM4        |
| UART5  | /dev/ttyS4 | TELEM2        |
| USART6 | /dev/ttyS5 | PX4IO/RC      |
| UART7  | /dev/ttyS6 | TELEM1        |
| UART8  | /dev/ttyS7 | GPS2          |

## Building Firmware {#building_firmware}

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make atlas_fmu-v6x_default
```

## Debug Port {#debug_port}

<!-- placeholder
Which connectors carry the PX4 System Console and SWD (FMU and IO), whether they follow Pixhawk Debug Full or Mini, and, if not, the connector, pinout and cable.
Good examples: agam_v6xrt.md#debug_port, ark_v6xrt.md#debug_port, cuav_pixhawk_v6x.md#debug_port
-->

## Assembly {#assembly}

### Radio Control {#radio_control}

RC input is wired directly to the FMU, not through PX4IO.

- PPM/S.Bus: dedicated single-wire input on a timer capture pin, decoded by the FMU (`RC_SERIAL_SINGLEWIRE`).
- Spektrum/DSM: dedicated FMU input with a switched 3.3V power rail for the satellite receiver.

PX4IO's serial link (`USART6`/`/dev/ttyS5`) is the FMU↔IO communication bus used for PWM output and safety-switch status, and is unrelated to RC input decoding.

### GPS & Compass {#gps_compass}

<!-- placeholder
GPS1 and GPS2 as labelled, with connector and what each carries (GPS, compass, safety switch, buzzer, LED); the onboard magnetometer; a link to ../assembly/mount_gps_compass.md.
Good examples: ark_v6xrt.md#gps_compass, amovlab_flycore.md#gps_compass
-->

## Further Information {#further_information}

- [Allocor Atlas Product Page](https://www.allocor.tech/atlas-product-page)
