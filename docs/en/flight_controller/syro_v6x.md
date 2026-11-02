# Syro V6X

<Badge type="tip" text="main (PX4 v2.0)" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](#store) for hardware support or compliance issues.
:::

The Syro V6X is a flight controller based on the FMUv6X reference design, with its own sensor set and PX4 board target.
It consists of a flight controller module with the STM32H753 main processor and sensors, and a baseboard that carries the PX4IO coprocessor, the Ethernet PHY and the external connectors.

<!-- TODO(Syro): add a product photo, stored under docs/assets/flight_controller/syro_v6x/, e.g.
![Syro V6X flight controller](../../assets/flight_controller/syro_v6x/syro_v6x_top.png) -->

::: info
This flight controller is intended for inclusion in the [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md) board list.
:::

## Specifications {#specifications}

- **Processors**
  - **Main FMU processor:** STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2 MB flash, 1 MB RAM)
  - **IO processor:** STM32F103 (PX4IO), on the baseboard
- **Sensors**
  - **IMU:** 2x InvenSense ICM-45686 (SPI1, SPI2), Bosch BMI088 (SPI3)
  - **Barometer:** 2x Bosch BMP581 (I2C2, I2C4)
  - **Magnetometer:** Bosch BMM350 (I2C4)
- **Storage**
  - FRAM (FM25V02A, SPI5) for parameters
  - microSD card (SDMMC2) for logs
  - EEPROMs on the module and baseboard holding the hardware IDs (see [Hardware Identification](#hardware_id))
- **Interfaces**
  - **PWM outputs:** 16 (8 from PX4IO, 8 from the FMU)
  - **Serial ports:** 6 for peripherals, plus the debug console
  - **Telemetry ports:** 3 (`TELEM1`, `TELEM2`, `TELEM3`), all with flow control
  - **GPS ports:** 2 (`GPS1` with safety switch and buzzer, `GPS2`)
  - **CAN buses:** 2
  - **Ethernet:** 100 Mbit/s (Microchip LAN8742A PHY)
  - **External SPI:** SPI6, with two chip selects and two data-ready inputs
  - **Power module inputs:** 2 digital (I2C) power module inputs
  - **Analog inputs:** 2 (3.3 V and 6.6 V range)
  - **RC input:** Yes, on the PX4IO
  - **USB:** Yes
- **Other**
  - IMU heater output <!-- TODO(Syro): confirm the heater is fitted; the heater GPIO (PB10) is inherited from the FMUv6X design. -->
  - Buzzer and safety switch on `GPS1`
- **Electrical data**
  <!-- TODO(Syro): input voltage range and maximum current of the power module inputs and USB, and the servo rail voltage range. -->
- **Mechanical data**
  <!-- TODO(Syro): dimensions, weight and mounting-hole pattern of the module and the baseboard. -->

## Where to Buy {#store}

<!-- TODO(Syro): add the store link and a manufacturer contact (website or support email). -->

## Connections

<!-- TODO(Syro): list every physical connector exactly as labelled on the board, with the connector type (e.g. JST GH 1.25 mm, CLIK-Mate 2.0 mm), and say whether the pinouts follow the Pixhawk Connector Standard (DS-009). The firmware defines these ports:
- POWER1 and POWER2 (digital power modules)
- USB
- TELEM1, TELEM2, TELEM3
- GPS1 (with safety switch and buzzer), GPS2
- UART4 & I2C
- CAN1, CAN2
- Ethernet
- SPI6 (external SPI)
- RC IN (PX4IO), and DSM if fitted
- MAIN 1-8 (PX4IO) and AUX 1-8 (FMU) PWM outputs, servo rail input
- ADC / IO (analog inputs, capture input)
- FMU debug and IO debug
- microSD card slot -->

## Pinouts

<!-- TODO(Syro): add a pin table for every connector (pin, signal, voltage), a pin 1 orientation image, and a connector layout image.
The board support guide requires a public pinout mapping PX4 pin definitions to the microcontroller pins and physical connectors. -->

## Block Diagram

<!-- TODO(Syro): add a block diagram or schematic of the main parts (processors, sensors, power supply and power path).
It must be detailed enough to understand the boot order and software requirements. -->

## Serial Port Mapping

| UART   | Device     | Port          | Flow Control |
| ------ | ---------- | ------------- | :----------: |
| USART1 | /dev/ttyS0 | `GPS1`        |      No      |
| USART2 | /dev/ttyS1 | `TELEM3`      |     Yes      |
| USART3 | /dev/ttyS2 | Debug Console |      No      |
| UART4  | /dev/ttyS3 | `UART4 & I2C` |      No      |
| UART5  | /dev/ttyS4 | `TELEM2`      |     Yes      |
| USART6 | /dev/ttyS5 | PX4IO/RC      |      No      |
| UART7  | /dev/ttyS6 | `TELEM1`      |     Yes      |
| UART8  | /dev/ttyS7 | `GPS2`        |      No      |

::: info
The mapping above applies to the running PX4 firmware.
The Syro V6X bootloader enables only `UART5` (`TELEM2`), at 1500000 baud.
When flashing firmware over a UART with [`px4_uploader.py`](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4_uploader.py) you must connect to **TELEM2**.
:::

## I2C Buses

| Bus  | Type     | Connected to                                                           |
| ---- | -------- | ---------------------------------------------------------------------- |
| I2C1 | External | `GPS1` (compass), `POWER1` (power monitor)                             |
| I2C2 | External | `GPS2`, `POWER2` (power monitor), onboard BMP581 barometer (`0x46`)    |
| I2C3 | External | `UART4 & I2C`, baseboard EEPROM (`0x51`)                               |
| I2C4 | Internal | BMM350 magnetometer, BMP581 barometer (`0x47`), module EEPROM (`0x50`) |

Do not connect external devices that use address `0x46` to I2C2, or address `0x51` to I2C3, as these addresses are used by onboard devices.

## Power {#power}

The flight controller is powered from the `POWER1` and `POWER2` digital power module inputs, or from USB.
If more than one source is connected, the board selects one in the order `POWER1`, `POWER2`, USB.

PX4 supports INA226, INA228 and INA238 based digital power modules on these inputs.
Enable the matching driver with [SENS_EN_INA226](../advanced_config/parameter_reference.md#SENS_EN_INA226), [SENS_EN_INA228](../advanced_config/parameter_reference.md#SENS_EN_INA228) or [SENS_EN_INA238](../advanced_config/parameter_reference.md#SENS_EN_INA238).
If none of these parameters is set, PX4 tries to detect a supported power monitor automatically.

<!-- TODO(Syro): input voltage limits, typical power consumption, and the voltage rating of the servo rail. -->

::: warning
The PWM output ports are not powered by the power module inputs.
The output rail must be [separately powered](../peripherals/pwm_escs_and_servo.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for multicopters.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

## Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The RC input is connected to the PX4IO coprocessor.
For PPM and S.Bus receivers, a single signal wire carries all channels.
If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a PPM encoder.
For more information, see [RC receivers](../getting_started/rc_transmitter_receiver.md).

<!-- TODO(Syro): list the RC protocols supported on the RC IN connector, and whether a DSM connector is fitted. -->

## GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the `GPS1` and `GPS2` ports.
GPS modules should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

`GPS1` includes the safety switch and buzzer signals.
By default PX4 starts an IST8310 compass on `GPS1`, as used on Holybro GPS modules.
The common magnetometer drivers are also included, so other supported GPS/compass modules are detected.

## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, or **TELEM3**.
If connected to **TELEM1**, no further configuration is required.
The other radio is connected to your ground station computer or mobile device (usually by USB).

## Ethernet

The board has a 100 Mbit/s Ethernet port.
By default PX4 runs a MAVLink instance on Ethernet, broadcasting on UDP port 14550.

See [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md) for the network configuration, and [uXRCE-DDS](../middleware/uxrce_dds.md) for using Ethernet to connect to ROS 2 on a companion computer.

## PWM Outputs {#pwm_outputs}

This flight controller supports up to 16 PWM outputs:

- 8 `MAIN` outputs, driven by the PX4IO coprocessor.
- 8 `AUX` outputs, driven by the FMU.

`AUX` outputs 1-6 support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
`AUX` outputs 7-8 and all `MAIN` outputs do not support DShot.

The 8 `AUX` outputs are split into 3 groups:

- Outputs 1-4 are in group 1, using Timer 5.
- Outputs 5-6 are in group 2, using Timer 4.
- Outputs 7-8 are in group 3, using Timer 12.

All outputs within the same group must use the same output protocol and rate.

`AUX` output 5 can also be used as a PWM input.

## SD Card

The board has a microSD card slot, used for flight logs.

## Debug Port

The PX4 system console runs on `USART3` at 57600 baud.

<!-- TODO(Syro): name the debug connector(s) for the FMU and the PX4IO, their connector type and pinout, and whether SWD is available. -->

## Hardware Identification {#hardware_id}

The flight controller module and the baseboard each store a hardware ID in an EEPROM.
PX4 reads these at boot to identify the board and the hardware fitted on the baseboard:

- **Baseboard ID:** `0x110` (stored in the baseboard EEPROM on I2C3).
  It tells PX4 that the PX4IO, CAN2, the second power module input and Ethernet are present.
- **Module revision:** `0x10` (stored in the module EEPROM on I2C4).

Both values are written in production with the `mft_cfg` command:

```sh
mft_cfg set hwver -i 0x110
mft_cfg set hwrev -i 0x10
```

::: warning
If the baseboard EEPROM is blank, PX4 reports the board as not supported and starts without the PX4IO (`MAIN` outputs), CAN2, the second power module input and Ethernet.
:::

## Building Firmware

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make syro_fmuv6x_default
```

Build and upload via USB:

```sh
make syro_fmuv6x_default upload
```

Build the bootloader:

```sh
make syro_fmuv6x_bootloader
```

## Bootloader / Board ID

- **Board ID:** 110 (`boards/syro/fmuv6x/firmware.prototype`)
- **USB VID/PID:** `0x3643` / `0x0001`
- **Bootloader USB product string:** `Syro BL FMU v6X`

## Supported Platforms / Airframes

Any multicopter, airplane, VTOL, rover or boat that can be controlled with normal RC servos or Futaba S-Bus servos.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Maintenance

- **Vendor:** Syro
- **PX4 board target:** `syro_fmuv6x_default`
- **PX4 hardware type prefix:** `SyroV6X`
