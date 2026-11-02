# ARK Electronics ARKV6X

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://arkelectron.com/contact-us/) for hardware support or compliance issues.
:::

The USA-built [ARKV6X](https://arkelectron.gitbook.io/ark-documentation/flight-controllers/arkv6x) flight controller is based on the [FMUv6X and Pixhawk Autopilot Bus open source standards](https://github.com/pixhawk/Pixhawk-Standards).

With triple synced IMUs, data averaging, voting, and filtering are possible.
The Pixhawk Autopilot Bus (PAB) form factor enables the ARKV6X to be used on any [PAB-compatible carrier board](../flight_controller/pixhawk_autopilot_bus.md), such as the [ARK Pixhawk Autopilot Bus Carrier](../flight_controller/ark_pab.md).

![ARKV6X Main Photo](../../assets/flight_controller/arkv6x/ark_v6x_front.jpg)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Specifications {#specifications}

### Processor {#processor}

- **Main FMU processor:** [STM32H743IIK6](https://www.st.com/en/microcontrollers-microprocessors/stm32h743ii.html) (480 MHz, 2 MB flash, 1 MB RAM)

### Sensors {#sensors}

- **IMU:** 2× [InvenSense ICM-42688-P](https://www.invensense.tdk.com/en-us/products/6-axis/icm-42688-p), [InvenSense IIM-42652](https://www.invensense.tdk.com/en-us/products/6-axis/iim-42652) (industrial)
- **Barometer:** [Bosch BMP390](https://www.bosch-sensortec.com/en/products/environmental-sensors/pressure-sensors/bmp390/)
- **Magnetometer:** [Bosch BMM150](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf)
- **Heater:** 1 W, keeps the sensors warm in extreme conditions

### Interfaces {#interfaces}

- **Parameter storage:** FRAM
- **SD card:** MicroSD slot
- **LED indicators:** Yes

### Electrical Data {#electrical_data}

- **Input voltage:** 5 V
- **Current draw:** 500 mA (300 mA main system, 200 mA heater)

### Mechanical Data {#mechanical_data}

- **Dimensions:** 3.6 x 2.9 x 0.5 cm
- **Weight:** 5.0 g
- **Form factor:** [Pixhawk Autopilot Bus (PAB)](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)

## Where to Buy {#store}

Order from [ARK Electronics](https://arkelectron.com/product/arkv6x/) (US).

## Pinouts {#pinouts}

For the pinout of the ARKV6X see the [DS-010 Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf).

## Serial Port Mapping {#serial_port_mapping}

| UART   | Device     | Port          |
| ------ | ---------- | ------------- |
| USART1 | /dev/ttyS0 | `GPS`         |
| USART2 | /dev/ttyS1 | `TELEM3`      |
| USART3 | /dev/ttyS2 | Debug Console |
| UART4  | /dev/ttyS3 | `UART4 & I2C` |
| UART5  | /dev/ttyS4 | `TELEM2`      |
| USART6 | /dev/ttyS5 | PX4IO/`RC`    |
| UART7  | /dev/ttyS6 | `TELEM1`      |
| UART8  | /dev/ttyS7 | `GPS2`        |

::: info
The mapping above applies to the running PX4 firmware.
The ARKV6X bootloader enables only `UART7` (`TELEM1`), so when flashing firmware over UART with [`px4_uploader.py`](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4_uploader.py) you must connect to the `TELEM1` port — no other UART will respond in bootloader mode.
:::

## Building Firmware {#building_firmware}

```sh
make ark_fmu-v6x_default
```

## Further Information {#further_information}

- [ARK Electronics ARKV6X](https://arkelectron.gitbook.io/ark-documentation/flight-controllers/arkv6x) (ARK Docs)
