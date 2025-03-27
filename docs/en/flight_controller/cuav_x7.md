# CUAV X7 Flight Controller

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.cuav.net) for hardware support or compliance issues.
:::

The [X7](http://doc.cuav.net/flight-controller/x7/en/x7.html)<sup>&reg;</sup> flight controller is a high-performance autopilot.
It is an ideal choice for industrial drones and large-scale heavy-duty drones.
It is mainly supplied to commercial manufacturers.

![CUAV x7](../../assets/flight_controller/cuav_x7/x7.jpg)

The flight controller adopts a modular design and can be matched with different base plates.
You can design a dedicated carrier board for your UAV to improve the integration of commercial systems, reduce wiring, improve system reliability, and enhance your UAV competitiveness (for example, integrating airspeed sensors, telemetry or even a companion computer, in the carrier board).
CUAV has also provided a variety of carrier boards for you to choose from.

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Features

- Internal shock absorption
- Modular design, can be DIY carrier board
- Support USB_HS, download logs faster (PX4 not yet supported)
- Support more DShot output
- Support IMU heating, make the sensor work better
- Dedicated CAN battery port
- 3 sets of IMU sensors
- Car-grade RM3100 compass
- High performance processor

:::tip
The manufacturer [CUAV Docs](https://doc.cuav.net/flight-controller/x7/en/) are the canonical reference for the X7.
They should be used by preference as they contain the most complete and up to date information.
:::

## Quick Summary

- Main FMU Processor: STM32H743
- On-board sensors:

  - Accelerometer/Gyroscope: ICM-20689
  - Accelerometer/Gyroscope: ICM-20649
  - Accelerometer/Gyroscope: BMI088
  - Magnetometer: RM3100
  - Barometer: MS5611\*2

- Interfaces:
  - 14 PWM outputs （12 supports Dshot）
  - Support multiple RC inputs (SBUs / CPPM / DSM)
  - Analogue / PWM RSSI input
  - 2 GPS ports(GPS and UART4 ports)
  - 4 i2c buses(Two i2c dedicated ports)
  - 2 CAN bus ports
  - 2 Power ports(Power A is common adc interface, Power C is DroneCAN battery interface)
  - 2 ADC input
  - 1 USB ports
- Power System:
  - Power: 4.3~5.4V
  - USB Input: 4.75~5.25V
  - Servo Rail Input: 0~36V
- Weight and Dimensions:
  - Weight: 101 g
- Other Characteristics:
  - Operating temperature: -20 ~ 80°c（Measured value）
  - Three imus
  - Supports temperature compensation
  - Internal shock absorption

::: info
When it runs PX4 firmware, only 8 pwm works, the remaining 6 pwm are still being adapted, so it is not compatible with VOLT now.
:::

## Where to Buy

[CUAV Store](https://store.cuav.net)

[CUAV aliexpress](https://www.aliexpress.com/item/4001042683738.html?spm=a2g0o.detail.1000060.2.1ebb2a9d3WDryi&gps-id=pcDetailBottomMoreThisSeller&scm=1007.13339.169870.0&scm_id=1007.13339.169870.0&scm-url=1007.13339.169870.0&pvid=f0df2481-1c0a-44eb-92a4-9c11c6cb3d06&_t=gps-id:pcDetailBottomMoreThisSeller,scm-url:1007.13339.169870.0,pvid:f0df2481-1c0a-44eb-92a4-9c11c6cb3d06,tpp_buckets:668%230%23131923%2320_668%23808%234094%23518_668%23888%233325%2319_668%234328%2319934%23630_668%232846%238115%23807_668%232717%237566%23827_668%231000022185%231000066058%230_668%233468%2315607%2376)

## Connections (Wiring)

[CUAV X7 Wiring Quickstart](http://doc.cuav.net/flight-controller/x7/en/quick-start/quick-start-x7.html)

## Size and Pinouts

![CUAV x7](../../assets/flight_controller/cuav_x7/x7-size.jpg)

![X7 pinouts](../../assets/flight_controller/cuav_x7/x7-pinouts.jpg)

:::warning
The `RCIN` port is limited to powering the RC receiver and cannot be connected to any power/load.
:::

## Voltage Ratings

The _X7 AutoPilot_ can be triple-redundant on the power supply if three power sources are supplied.
The power rails are: **POWERA**, **POWERC** and **USB**.

::: info
The output power rails **PWM OUT** (0V to 36V) do not power the flight controller board (and are not powered by it).
You must supply power to one of **POWERA**, **POWERC** or **USB** or the board will be unpowered.
:::

**Normal Operation Maximum Ratings**

Under these conditions all power sources will be used in this order to power the system:

1. **POWERA** and **POWERC** inputs (4.3V to 5.4V)
2. **USB** input (4.75V to 5.25V)

## Building Firmware

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make cuav_x7pro_default
```

## Over Current Protection

The _X7_ has over-current protection on the 5 Volt Peripheral and 5 Volt high power, which limits the current to 2.5A.
The _X7_ has short circuit protection.

:::warning
Up to 2.5 A can be delivered to the connectors listed as pin 1 (although these are only rated at 1 A).
:::

## Debug Port

The system's serial console and SWD interface operate on the **DSU7** port.
Simply connect the FTDI cable to the DSU7 connector (the product list contains the CUAV FTDI cable).

![Debug port (DSU7)](../../assets/flight_controller/cuav_v5_plus/debug_port_dsu7.jpg)

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) operate on the **FMU Debug** port (`DSU7`).

The debug port (`DSU7`) uses a [JST BM06B](https://www.digikey.com.au/product-detail/en/jst-sales-america-inc/BM06B-GHS-TBT-LF-SN-N/455-1582-1-ND/807850) connector and has the following pinout:

| Pin     | Signal         | Volt  |
| ------- | -------------- | ----- |
| 1 (red) | 5V+            | +5V   |
| 2 (blk) | DEBUG TX (OUT) | +3.3V |
| 3 (blk) | DEBUG RX (IN)  | +3.3V |
| 4 (blk) | FMU_SWDIO      | +3.3V |
| 5 (blk) | FMU_SWCLK      | +3.3V |
| 6 (blk) | GND            | GND   |

CUAV provides a dedicated debugging cable, which can be connected to the `DSU7` port.
This splits out an FTDI cable for connecting the [PX4 System Console](../debug/system_console.md) to a computer USB port, and SWD pins used for SWD/JTAG debugging.
The provided debug cable does not connect to the SWD port `Vref` pin (1).

![CUAV Debug cable](../../assets/flight_controller/cuav_v5_plus/cuav_v5_debug_cable.jpg)

:::warning
The SWD Vref pin (1) uses 5V as Vref but the CPU is run at 3.3V!

Some JTAG adapters (SEGGER J-Link) will use the Vref voltage to set the voltage on the SWD lines.
For direct connection to _Segger Jlink_ we recommended you use the 3.3 Volts from pin 4 of the connector marked `DSM`/`SBUS`/`RSSI` to provide `Vtref` to the JTAG (i.e. providing 3.3V and _NOT_ 5V).
:::

## Supported Platforms / Airframes

Any multicopter / airplane / rover or boat that can be controlled with normal RC servos or Futaba S-Bus servos.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Further info

- [Quick start](http://doc.cuav.net/flight-controller/x7/en/quick-start/quick-start-x7.html)
- [CUAV docs](http://doc.cuav.net)
- [x7 schematic](https://github.com/cuav/hardware/tree/master/X7_Autopilot)
