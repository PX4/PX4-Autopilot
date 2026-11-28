# ZeroOne X9

<Badge type="tip" text="PX4 main (v2.0)" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://01aero.com/) for hardware support or compliance issues.
:::

The [ZeroOne X9](https://01aero.com/product/x9/) is a modular flight controller manufactured by ZeroOne.

<!-- placeholder
What the board is and what distinguishes it.
The X9 is an X9 Core module (two DF17 board-to-board connectors) on an interface baseboard; the X9 Core is also sold on its own for custom baseboards.
Say whether it is based on a reference design (such as FMUv6X) only if the manufacturer claims it.
Hero image: a local photo under ../../assets/flight_controller/zeroone_x9/
Good example: ark_v6x.md
-->

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Specifications {#specifications}

### Processor {#processor}

- **Main FMU processor:** [STM32H753](https://www.st.com/en/microcontrollers-microprocessors/stm32h753ii.html) (Arm Cortex-M7, 480MHz, 2MB flash, 1MB RAM)
- **IO processor:** STM32F103 ([PX4IO](../hardware/reference_design.md#main-io-function-breakdown))

### Sensors {#sensors}

- **IMU:** 3× [InvenSense IIM-42652](https://www.invensense.tdk.com/en-us/products/6-axis/iim-42652) (SPI1, SPI2, SPI3), with omnidirectional vibration damping
- **Barometer:** 2× TDK ICP-20100, or Bosch BMP581 and SPL07 (TBD: which bus each is on)
- **Magnetometer:** PNI RM3100 (I2C4)
- **Heater:** IMU heater (TBD: power)

### Interfaces {#interfaces}

- **PWM outputs:** 14 on the X9 baseboard: 8 PX4IO (MAIN) and 6 FMU (AUX) (see [PWM Outputs](#pwm_outputs))
- **Serial ports:** 7 (`GPS1`, `GPS2`, `TELEM1`, `TELEM2`, `TELEM3`, `UART4`, Debug Console) (TBD: which are on the X9 baseboard)
- **I2C buses:** 4 (I2C4 internal, the rest external)
- **SPI buses:** 5, of which SPI6 is external with 2 chip selects and 2 data-ready lines
- **CAN buses:** 2
- **Ethernet:** TBD (speed)
- **USB:** USB-C
- **RC input:** `SBUS` (TBD: DSM and PPM)
- **Parameter storage:** FRAM on SPI5
- **SD card:** TBD
- **LED indicators:** Yes

### Electrical Data {#electrical_data}

- **Input voltage:** 4.5V to 5.4V
- **Current draw:** TBD
- **Power monitoring:** 2 power inputs (`Power1`, `Power2`) (TBD: monitor type)

### Mechanical Data {#mechanical_data}

- **Dimensions:**
  - X9: 38.8 × 38.8 × 38.8mm
  - X9 Core: 38.8 × 38.8 × 26.5mm
- **Mounting hole spacing:** 32 × 32mm (TBD: hole diameter)
- **Weight:**
  - X9: 72g
  - X9 Core: 57.5g
- **Form factor:** X9 Core module with two 80-pin Hirose DF17 board-to-board connectors, on an interface baseboard
- **Operating temperature:** -20°C to 85°C

## Where to Buy {#store}

<!-- placeholder: purchase link -->

## Pinouts {#pinouts}

<!-- placeholder
Connector pinout tables for the X9 baseboard and the X9 Core DF17 connectors, or a link to a public pinout document.
Required by the board support guide: also add a block diagram or schematic of the main components (sensors, power supply).
Good example: ark_v6x.md#pinouts
-->

## Power {#power}

<!-- placeholder
Power1 and Power2 inputs (BOARD_NUMBER_BRICKS 2), 2 digital monitors (BOARD_HAS_NBAT_V/I 2d).
Firmware can start INA226/INA228/INA238 on I2C1 and I2C2 (SENS_EN_INA226/228/238); none is enabled by default.
The manufacturer pinout shows the power connectors carrying CAN, not I2C: say which power modules to use and how to configure them.
-->

## Voltage Ratings {#voltage_ratings}

<!-- placeholder: input voltage ranges for Power1, Power2, USB and the servo rail, and which inputs are redundant -->

## PWM Outputs {#pwm_outputs}

<!-- placeholder
MAIN 1-8: PX4IO outputs (no DShot).
AUX 1-6: FMU outputs. All support DShot and bidirectional DShot.
Groups: AUX 1-4 (Timer5), AUX 5-6 (Timer4). Outputs in a group must use the same protocol and rate.
FMU outputs 7-8 (Timer12, no DShot) and the FMU_CAP1 capture channel (Timer1) are on the X9 Core connector only.
PWM output voltage is selectable between 3.3V and 5V with PWM_VOLT_SEL (reboot required).
Good example: ark_v6x.md#pwm_outputs
-->

## Ethernet {#ethernet}

<!-- placeholder
Ethernet port, and the default MAVLink-over-Ethernet configuration (MAV_2_CONFIG 1000, UDP port 14550, from rc.board_defaults).
-->

## SD Card (Optional) {#sd_card}

<!-- placeholder: card type and maximum supported size -->

## Serial Port Mapping {#serial_port_mapping}

| UART   | Device     | Port          |
| ------ | ---------- | ------------- |
| USART1 | /dev/ttyS0 | GPS1          |
| USART2 | /dev/ttyS1 | TELEM3        |
| USART3 | /dev/ttyS2 | Debug Console |
| UART4  | /dev/ttyS3 | UART4         |
| UART5  | /dev/ttyS4 | TELEM2        |
| USART6 | /dev/ttyS5 | PX4IO         |
| UART7  | /dev/ttyS6 | TELEM1        |
| UART8  | /dev/ttyS7 | GPS2          |

<!-- placeholder: which ports have flow control (firmware enables it on TELEM1, TELEM2 and TELEM3), and which ports are only on the X9 Core connector -->

## Assembly {#assembly}

<!-- placeholder: link to a quick-start or wiring guide, if there is one -->

### Radio Control {#radio_control}

<!-- placeholder
Which connector RC receivers go to, and which protocols each supports.
RC goes through the PX4IO; the X9 Core also has a PPM input pin (PI5).
DSM bind pulses go out on the PPM pin: say which connector it reaches.
Good example: ark_v6x.md#radio_control
-->

### GPS & Compass {#gps_compass}

<!-- placeholder: which connectors a GPS and external compass use (the GPS1 port carries UART only; the compass connects to I2C1) -->

### Telemetry Radios (Optional) {#telemetry}

<!-- placeholder: which TELEM port to use for a telemetry radio -->

### CAN {#can}

<!-- placeholder: CAN1 and CAN2 connectors, and how to enable DroneCAN -->

## Building Firmware {#building_firmware}

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make zeroone_x9_default
```

## Debug Port {#debug_port}

<!-- placeholder: the connector carrying the debug console (USART3) and SWD, with its pinout -->

## Further Information {#further_information}

- [X9 manual](https://docs.01aero.cn/docs/JfRLkT8E) (ZeroOne, Chinese)
