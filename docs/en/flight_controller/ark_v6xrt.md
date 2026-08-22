# ARK Electronics ARKV6X-RT

<Badge type="tip" text="main (PX4 v2.0)" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://arkelectron.com/contact-us/) for hardware support or compliance issues.
:::

The USA-built ARKV6X-RT flight controller is an NXP i.MX RT1176 variant of the [ARKV6X](../flight_controller/ark_v6x.md), based on the [FMUv6X-RT and Pixhawk Autopilot Bus open source standards](https://github.com/pixhawk/Pixhawk-Standards).

With triple synced IMUs, data averaging, voting, and filtering is possible.
The Pixhawk Autopilot Bus (PAB) form factor enables the ARKV6X-RT to be used on any [PAB-compatible carrier board](../flight_controller/pixhawk_autopilot_bus.md), such as the [ARK Pixhawk Autopilot Bus Carrier](../flight_controller/ark_pab.md).

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Where To Buy {#store}

Order From [Ark Electronics](https://arkelectron.com/) (US)

## Sensors

- [Invensense ICM-45686 IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-45686/)
- [Invensense IIM-20670 Industrial IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/iim-20670/)
- [ST LSM6DSV80X High-g IMU](https://www.st.com/en/mems-and-sensors/lsm6dsv80x.html)
- [Bosch BMP390 Barometer](https://www.bosch-sensortec.com/en/products/environmental-sensors/pressure-sensors/bmp390/)
- [ST IIS2MDC Magnetometer](https://www.st.com/en/mems-and-sensors/iis2mdc.html)

Each IMU sits on its own SPI bus with an independent power rail and data-ready line:

| IMU        | Bus  | Full scale          | Publish rate |
| ---------- | ---- | ------------------- | ------------ |
| ICM-45686  | SPI1 | ±32 g / ±4000 dps   | 800 Hz       |
| IIM-20670  | SPI2 | ±65.5 g / ±1966 dps | 1000 Hz      |
| LSM6DSV80X | SPI3 | ±80 g / ±4000 dps   | 768 Hz       |

Start order sets the sensor instance and the voter prefers the lowest instance at equal priority, so the ICM-45686 is the primary.
The IIM-20670 is started last deliberately: its on-chip gyro low-pass cannot be set above 60 Hz, which is too much group delay for the rate loop, so it serves as a navigation and fallback IMU.

The LSM6DSV80X publishes its ±80 g high-g accelerometer rather than the ±16 g channel, so the range never changes in flight.
That channel's zero-g offset drifts roughly 2 mg/°C — about 30× the other two IMUs — so `HEATER1_SENS_ID` defaults to this die.
Run the accelerometer calibration only after `heater status` reports the setpoint has been reached, and expect `Accel 1 inconsistent` on a cold board until the heater catches up.

## Microprocessor

- [NXP i.MX RT1176 MCU](https://www.nxp.com/products/i.MX-RT1170)
  - Cortex-M7 at 996 MHz (the second Cortex-M4 core is unused by PX4)
  - No internal flash: code executes in place from a 64 MB external octal NOR, of which PX4 uses the first 4 MB
  - 2 MB RAM

## Other Features

- FRAM for parameter storage
- [Pixhawk Autopilot Bus (PAB) Form Factor](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)
- Ethernet
- 2x CAN
- LED Indicators
- MicroSD Slot
- USA Built
- Sensor heater for operation in extreme conditions

## Pinout

For pinout of the ARKV6X-RT see the [DS-10 Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)

## Serial Port Mapping

| UART     | Device     | Port          |
| -------- | ---------- | ------------- |
| LPUART1  | /dev/ttyS0 | Debug Console |
| LPUART3  | /dev/ttyS1 | GPS           |
| LPUART4  | /dev/ttyS2 | TELEM1        |
| LPUART5  | /dev/ttyS3 | GPS2          |
| LPUART6  | /dev/ttyS4 | PX4IO/RC      |
| LPUART8  | /dev/ttyS5 | TELEM2        |
| LPUART10 | /dev/ttyS6 | TELEM3        |
| LPUART11 | /dev/ttyS7 | EXT2          |

::: info
The mapping above applies to the running PX4 firmware.
The ARKV6X-RT bootloader enables only `LPUART8` (TELEM2), so when flashing firmware over UART with [`px4_uploader.py`](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4_uploader.py) you must connect to the `TELEM2` port — no other UART will respond in bootloader mode.
:::

## Building Firmware

```sh
make ark_fmu-v6xrt_default
```

Uploading over USB works as it does on any other board once a bootloader is present.
A board with no bootloader — or a wiped NOR — has to be recovered over SWD, which needs NXP's device family pack for the RT1176 flash algorithm and a FlexSPI preconditioning step; see [`boards/ark/fmu-v6xrt/README.md`](https://github.com/PX4/PX4-Autopilot/blob/main/boards/ark/fmu-v6xrt/README.md) for the procedure.

## See Also

- [ARK Electronics ARKV6X](../flight_controller/ark_v6x.md)
