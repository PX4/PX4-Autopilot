# ARK Electronics ARKV6X

<Badge type="tip" text="PX4 v1.13" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://arkelectron.com/contact-us/) for hardware support or compliance issues.
:::

The USA-built [ARKV6X](https://docs.arkelectron.com/flight-controllers/arkv6x) flight controller is based on the [FMUv6X and Pixhawk Autopilot Bus open source standards](https://github.com/pixhawk/Pixhawk-Standards).

With triple synced IMUs, data averaging, voting, and filtering are possible.
The Pixhawk Autopilot Bus (PAB) form factor enables the ARKV6X to be used on any [PAB-compatible carrier board](../flight_controller/pixhawk_autopilot_bus.md), such as the [ARK Pixhawk Autopilot Bus Carrier](../flight_controller/ark_pab.md).
The ARKV6X must be used with a carrier board, which provides all of its connectors for power, peripherals and outputs.

![ARKV6X Main Photo](../../assets/flight_controller/arkv6x/ark_v6x_front.jpg)

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Specifications {#specifications}

### Processor {#processor}

- **Main FMU processor:** [STM32H743IIK6](https://www.st.com/en/microcontrollers-microprocessors/stm32h743ii.html) (Arm Cortex-M7, 480 MHz, 2 MB flash, 1 MB RAM)
- **IO processor:** PX4IO, on carriers that fit one

### Sensors {#sensors}

- **IMU:**
  - ARKV6X: 2× [InvenSense ICM-42688-P](https://www.invensense.tdk.com/en-us/products/6-axis/icm-42688-p) (SPI2, SPI3), [InvenSense IIM-42652](https://www.invensense.tdk.com/en-us/products/6-axis/iim-42652) (industrial, SPI1)
  - ARKV6X Extended Range: 3× [InvenSense IIM-42653](https://invensense.tdk.com/products/smartindustrial/iim-42653/) (SPI1, SPI2, SPI3), for a 4000 dps gyroscope and 32 g accelerometer range
- **Barometer:** [Bosch BMP390](https://www.bosch-sensortec.com/en/products/environmental-sensors/pressure-sensors/bmp390/) (I2C4)
- **Magnetometer:** [Bosch BMM150](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf) (I2C4)
- **Heater:** 1 W, keeps the sensors warm in extreme conditions. It regulates on the temperature of the IIM-42652 (IIM-42653 on the Extended Range).

### Interfaces {#interfaces}

- **PWM outputs:** 8 FMU outputs, plus 8 from PX4IO on carriers that fit one (see [PWM Outputs](#pwm_outputs))
- **Serial ports:** 8 (`GPS1`, `GPS2`, `TELEM1`, `TELEM2`, `TELEM3`, `UART4 & I2C`, PX4IO/`RC`, Debug Console)
- **I2C buses:** 4 (I2C4 internal, the rest external)
- **SPI buses:** 5, of which SPI6 is external with 2 chip selects and 2 data-ready lines
- **CAN buses:** 2
- **Ethernet:** Yes
- **USB:** Yes
- **RC input:** Yes
- **Parameter storage:** FRAM (FM25V02A) on SPI5
- **SD card:** MicroSD slot
- **LED indicators:** Yes

### Electrical Data {#electrical_data}

- **Input voltage:** 5 V
- **Current draw:** 500 mA (300 mA main system, 200 mA heater)
- **Power monitoring:** Digital, up to 3 power monitors (INA226 by default)

### Mechanical Data {#mechanical_data}

- **Dimensions:** 3.6 x 2.9 x 0.5 cm
- **Weight:** 5.0 g
- **Form factor:** [Pixhawk Autopilot Bus (PAB)](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)
- **Operating temperature:** -10 °C to +60 °C

## Where to Buy {#store}

Order from [ARK Electronics](https://arkelectron.com/product/arkv6x/) (US).

## Pinouts {#pinouts}

For the pinout of the ARKV6X see the [DS-010 Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf).

## Power {#power}

The ARKV6X is powered with 5 V from its carrier board, through the PAB connector.

On the ARK PAB Carrier, power comes in on `POWER1` and `POWER2`, which are 6-pin Molex CLIK-Mate connectors.
The ARKV6X can be powered from `POWER1`, `POWER2` or USB, and selects between them automatically.

Battery monitoring is digital: both power ports carry I2C for a digital power module.
INA226 power monitors are enabled by default ([SENS_EN_INA226](../advanced_config/parameter_reference.md#SENS_EN_INA226)).
See [Battery and Power Module Setup](../config/battery.md) for configuration.

::: info
The PWM output port on the ARK PAB Carrier does not supply power (`VDD_SERVO` is not connected).
Servos and other hardware on the outputs must be [powered separately](../peripherals/pwm_escs_and_servo.md).
:::

## Voltage Ratings {#voltage_ratings}

The ARKV6X takes 5 V from its carrier through the PAB connector: 4.8 V minimum, 5.5 V absolute maximum.

On the ARK PAB Carrier, power is taken from `POWER1`, then `POWER2`, then USB.
The carrier has overvoltage protection at 5.8 V and undervoltage protection at 3.9 V.

## PWM Outputs {#pwm_outputs}

The ARKV6X provides 8 FMU PWM outputs, and 8 more from a PX4IO on carriers that fit one.
How many reach a connector depends on the carrier: the [ARK PAB Carrier](../flight_controller/ark_pab.md) has no PX4IO, and breaks out FMU outputs 1-8.
The firmware is built with PX4IO support, so the FMU outputs are labelled `AUX` in the actuator configuration, whichever carrier is used.

FMU outputs 1-6 support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
Outputs 7-8 do not support DShot.
PX4IO outputs do not support DShot.

The 8 FMU outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.

A ninth timer channel, `FMU_CAP` (Timer1), is intended for camera capture, and appears in the actuator configuration as _PWM Capture 1_.
On the ARK PAB Carrier it is on the `ADIO` port.

## Ethernet {#ethernet}

The ARKV6X supports Ethernet.
On the ARK PAB Carrier it is on the 4-pin JST-GH `ETH` port.

MAVLink instance 2 runs on Ethernet by default, on UDP port 14550.
See [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md) for how to configure it.

## SD Card (Optional) {#sd_card}

The ARKV6X has a MicroSD card slot.
For information about choosing and using an SD card see [SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).

## Serial Port Mapping {#serial_port_mapping}

| UART   | Device     | Port          | Flow Control |
| ------ | ---------- | ------------- | ------------ |
| USART1 | /dev/ttyS0 | `GPS1`        |              |
| USART2 | /dev/ttyS1 | `TELEM3`      | Yes          |
| USART3 | /dev/ttyS2 | Debug Console |              |
| UART4  | /dev/ttyS3 | `UART4 & I2C` |              |
| UART5  | /dev/ttyS4 | `TELEM2`      | Yes          |
| USART6 | /dev/ttyS5 | PX4IO/`RC`    |              |
| UART7  | /dev/ttyS6 | `TELEM1`      | Yes          |
| UART8  | /dev/ttyS7 | `GPS2`        |              |

::: info
The mapping above applies to the running PX4 firmware.
The ARKV6X bootloader enables only `UART7` (`TELEM1`), so when flashing firmware over UART with [`px4_uploader.py`](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4_uploader.py) you must connect to the `TELEM1` port — no other UART will respond in bootloader mode.
:::

## Assembly {#assembly}

The connectors the ARKV6X uses are on its carrier board.
The sections below describe the [ARK PAB Carrier](../flight_controller/ark_pab.md); see its page for the connector pinouts.

### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).
You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate.

How a receiver connects depends on the carrier:

- **ARK PAB Carrier** (no PX4IO): connect the receiver to the `RC/SBUS` port, which goes to the FMU (`USART6`, `/dev/ttyS5`).
  The firmware includes the [CRSF](../telemetry/crsf_telemetry.md), DSM, GHST and S.BUS drivers, but none is enabled by default.
  Enable the one for your receiver by setting its port parameter to `RC`: [RC_CRSF_PRT_CFG](../advanced_config/parameter_reference.md#RC_CRSF_PRT_CFG), [RC_DSM_PRT_CFG](../advanced_config/parameter_reference.md#RC_DSM_PRT_CFG), [RC_GHST_PRT_CFG](../advanced_config/parameter_reference.md#RC_GHST_PRT_CFG) or [RC_SBUS_PRT_CFG](../advanced_config/parameter_reference.md#RC_SBUS_PRT_CFG).
  Only one protocol can be enabled on the port.
  PPM receivers are not supported by the default firmware.
- **Carriers with a PX4IO**: connect the receiver to the carrier's RC input, which goes to the PX4IO.
  The PX4IO detects the [protocols it supports](../modules/modules_driver.md#px4io) automatically.
  CRSF and GHST receivers can't be decoded by the PX4IO, and must be connected to an FMU serial port instead.

### GPS & Compass {#gps_compass}

The ARK PAB Carrier has two GPS ports:

- `GPS1`: 10-pin JST-GH, with GPS (`USART1`), I2C, safety switch, safety LED and buzzer.
- `GPS2`: 6-pin JST-GH, with GPS (`UART8`) and I2C.

The ARKV6X has an onboard BMM150 magnetometer, but an external compass on the GPS module is recommended.
See [Mounting a GPS/Compass](../assembly/mount_gps_compass.md) for more information.

### Telemetry Radios (Optional) {#telemetry}

The ARK PAB Carrier has three [telemetry](../telemetry/index.md) ports, `TELEM1`, `TELEM2` and `TELEM3`, each a 6-pin JST-GH connector with flow control.
A telemetry radio connected to `TELEM1` needs no further configuration.

### CAN {#can}

The ARK PAB Carrier has two CAN ports, `CAN1` and `CAN2`, each a 4-pin JST-GH connector.
They can be used for [DroneCAN](../dronecan/index.md) peripherals, such as GPS modules, ESCs and power modules.

DroneCAN is not enabled by default: set [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to enable it.
DroneCAN ESCs are on `CAN2` by default ([UAVCAN_ESC_IFACE](../advanced_config/parameter_reference.md#UAVCAN_ESC_IFACE)).

## Building Firmware {#building_firmware}

::: tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make ark_fmu-v6x_default
```

## Debug Port {#debug_port}

The [PX4 System Console](../debug/system_console.md) (`USART3`) and [SWD interface](../debug/swd_debug.md) are on the carrier's FMU debug port.
On the ARK PAB Carrier this is a 10-pin JST-SH port that follows the [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) standard.

## Further Information {#further_information}

- [ARK Electronics ARKV6X](https://docs.arkelectron.com/flight-controllers/arkv6x) (ARK Docs)
- [ARKV6X Datasheet](https://github.com/ARK-Electronics/ark-hardware/blob/main/ARKV6X/datasheet/ARKV6X_Datasheet.pdf) (ARK)
- [ARKV6X 3D model](https://github.com/ARK-Electronics/ark-hardware/tree/main/ARKV6X/model) (ARK)
- [ARK Pixhawk Autopilot Bus Carrier](../flight_controller/ark_pab.md)
