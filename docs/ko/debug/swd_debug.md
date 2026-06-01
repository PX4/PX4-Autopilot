# SWD Debug Port

PX4 runs on ARM Cortex-M microcontrollers, which contain dedicated hardware for interactive debugging via the [_Serial Wire Debug (SWD)_][swd] interface and non-invasive profiling and high-bandwidth tracing via the [_Serial Wire Output (SWO)_][itm] and [_TRACE_ pins][etm].

The SWD debug interface allows direct, low-level, hardware access to the microcontroller's processor and peripherals, so it does not depend on any software on the device.
Therefore it can be used to debug bootloaders and operating systems such as NuttX.

## Debug Signals {#debug-signals}

Four signals are required for debugging (in bold) while the rest is recommended.

| 명칭                                                              | 형식    | 설명                                                                                                        |
| :-------------------------------------------------------------- | :---- | :-------------------------------------------------------------------------------------------------------- |
| **GND**                                                         | 전원    | Shared potential, common ground.                                                          |
| **VREF**                                                        | 전원    | The target reference voltage allows the debug probe to use level shifters on the signals. |
| **SWDIO**                                                       | I/O   | Serial Wire Debug data pin.                                                               |
| **SWCLK**                                                       | Input | Serial Wire Debug clock pin.                                                              |
| nRST                                                            | Input | The reset pin is optional (n = active low).                            |
| SWO                                                             | 출력    | Single wire trace asynchronous data out can output ITM and DWT data.                      |
| TRACECK                                                         | 출력    | Trace clock for parallel bus.                                                             |
| TRACED[0-3] | 출력    | Trace synchronous data bus with 1, 2, or 4 bits.                                          |

The hardware reset pin is optional, as most devices can also be reset via the SWD lines. However, quickly resetting the device via a button can be great for development.

The SWO pin can emit low-overhead, real-time profiling data with nanosecond timestamping and is therefore strongly recommended to have accessible for debugging.

The TRACE pins require specialized debug probes to deal with the high bandwidth and subsequent datastream decoding.
They are usually not accessible and are typically only used to debug very specific timing issues.

## Autopilot Debug Ports {#debug-ports}

Flight controllers commonly provide a single debug port that exposes both the [SWD Interface](#debug-signals) and [System Console](system_console.md).

The [Pixhawk Connector Standards](#pixhawk-standard-debug-ports) formalize the port that must be used in each FMU version.
However there are still many boards that use different pinouts or connectors, so we recommend you check the [documentation for your autopilot](../flight_controller/index.md) to confirm port location and pinout.

The debug port location and pinouts for a subset of autopilots are linked below:

<a id="port-information"></a>

| 오토파일럿                                                                                                   | 디버그 포트                                                                   |
| :------------------------------------------------------------------------------------------------------ | :----------------------------------------------------------------------- |
| [Holybro Pixhawk 6X-RT](../flight_controller/pixhawk6x-rt.md#debug_port) (FMUv6X-RT) | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [Holybro Pixhawk 6X](../flight_controller/pixhawk6x.md#debug_port) (FMUv6x)          | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [Holybro Pixhawk 5X](../flight_controller/pixhawk5x.md#debug_port) (FMUv5x)          | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [Holybro Durandal](../flight_controller/durandal.md#debug-port)                                         | [Pixhawk Debug Mini](#pixhawk-debug-mini)                                |
| [Holybro Pixhawk 4](../flight_controller/pixhawk4.md#debug_port) (FMUv5)             | [Pixhawk Debug Mini](#pixhawk-debug-mini)                                |
| [Holybro Pixhawk 6X Pro](../flight_controller/pixhawk6x_pro.md#debug-port)                              | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [Holybro Pixhawk 6C](../flight_controller/pixhawk6c.md#debug_port)                                      | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [Holybro Pixhawk 6C Mini](../flight_controller/pixhawk6c_mini.md#debug_port)                            | [Pixhawk Debug Mini](#pixhawk-debug-mini)                                |
| [Holybro Pix32 v6](../flight_controller/holybro_pix32_v6.md#debug_port)                                 | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [Holybro Pix32 v5](../flight_controller/holybro_pix32_v5.md#debug-port)                                 | [Pixhawk Debug Mini](#pixhawk-debug-mini)                                |
| [Holybro Kakute H7](../flight_controller/kakuteh7.md#debug-port)                                        | SWD pads and system console                                              |
| [Holybro Kakute H7 mini](../flight_controller/kakuteh7mini.md#debug-port)                               | SWD pads and system console                                              |
| [Holybro Kakute H7 V2](../flight_controller/kakuteh7v2.md#debug-port)                                   | SWD pads and system console                                              |
| [CUAV V5+](../flight_controller/cuav_v5_plus.md#debug-port)                                             | Custom port but comes with adaptor cable                                 |
| [CUAV V5nano](../flight_controller/cuav_v5_nano.md#debug_port)                                          | Custom port but comes with adaptor cable                                 |
| [CUAV Pixhawk V6X](../flight_controller/cuav_pixhawk_v6x.md#debug_port)                                 | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [CUAV X25-SUPER](../flight_controller/cuav_x25-super.md#debug_port)                                     | [Pixhawk Debug Mini] |
| [CUAV X25-EVO](../flight_controller/cuav_x25-evo.md#debug_port)                                         | [Pixhawk Debug Mini] |
| [CUAV Nora](../flight_controller/cuav_nora.md#debug-port)                                               | Custom port but comes with adaptor cable.                |
| [ARK Pixhawk Autopilot Bus Carrier](../flight_controller/ark_pab.md#debug-port)                         | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [NXP MR-VMU-RT1176](../flight_controller/nxp_mr_vmu_rt1176.md#debug_port)                               | [Pixhawk Debug Full](#pixhawk-debug-full)                                |
| [mRo Pixracer](../flight_controller/pixracer.md#debug-port)                                             | [Pixhawk Debug Mini](#pixhawk-debug-mini)                                |
| [S-Vehicle E2](../flight_controller/svehicle_e2.md#debug-port)                                          | [Pixhawk Debug Mini] |
| [AP-H743-R1](../flight_controller/x-mav_ap-h743r1.md#debug-port)                                        | 4-pin JST GH (SWD only)                               |
| [mRo Control Zero F7](../flight_controller/mro_control_zero_f7.md#debug_port)                           |                                                                          |

## Pixhawk Connector Standard Debug Ports {#pixhawk-standard-debug-ports}

The Pixhawk project has defines a standard pinout and connector type for different Pixhawk FMU releases:

:::tip
Check your [specific board](#port-information) to confirm the port used.
:::

| FMU 버전    | Pixhawk Version     | 디버그 포트                                    |
| :-------- | :------------------ | :---------------------------------------- |
| FMUv2     | Pixhawk / Pixhawk 1 | 10핀 ARM 디버그                               |
| FMUv3     | Pixhawk 2           | 6핀 SUR 디버그                                |
| FMUv4     | Pixhawk 3           | [Pixhawk Debug Mini](#pixhawk-debug-mini) |
| FMUv5     | Pixhawk 4 FMUv5     | [Pixhawk Debug Mini](#pixhawk-debug-mini) |
| FMUv5X    | Pixhawk 5X          | [Pixhawk Debug Full](#pixhawk-debug-full) |
| FMUv6     | Pixhawk 6           | [Pixhawk Debug Full](#pixhawk-debug-full) |
| FMUv6X    | Pixhawk 6X          | [Pixhawk Debug Full](#pixhawk-debug-full) |
| FMUv6X-RT | Pixhawk 6X-RT       | [Pixhawk Debug Full](#pixhawk-debug-full) |

:::info
There FMU and Pixhawk versions are (only) consistent after FMUv5X.
:::

### Pixhawk Debug Mini

The [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) defines the _Pixhawk Debug Mini_, a _6-Pin SH Debug Port_ that provides access to both SWD pins and the [System Console](system_console.md).

This is used in FMUv4 and FMUv5.

The pinout is as shown below (pins required for debugging are bold):

|  핀 | 신호         |
| -: | :--------- |
|  1 | **VREF**   |
|  2 | 콘솔 TX      |
|  3 | 콘솔 RX      |
|  4 | **SWDIO**  |
|  5 | **SWDCLK** |
|  6 | **GND**    |

The debug port definition includes the following solder pads (on board next to connector):

| 패드 | 신호    | 전압                    |
| -: | :---- | :-------------------- |
|  1 | nRST  | +3.3V |
|  2 | GPIO1 | +3.3V |
|  3 | GPIO2 | +3.3V |

The socket is a _6-pin JST SH_ - Digikey number: [BM06B-SRSS-TBT(LF)(SN)](https://www.digikey.com/en/products/detail/jst-sales-america-inc/BM06B-SRSS-TBT/1785724) (vertical mount), [SM06B-SRSS-TBT(LF)(SN)](https://www.digikey.com/en/products/detail/jst-sales-america-inc/SM06B-SRSS-TB/926712) (side mount).

You can connect to the debug port using a [cable like this one](https://www.digikey.com/en/products/detail/jst-sales-america-inc/A06SR06SR30K152A/6009379).

![6-pin JST SH Cable](../../assets/debug/cable_6pin_jst_sh.jpg)

### Pixhawk Debug Full

The [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) defines _Pixhawk Debug Full_, a _10-Pin SH Debug Port_ that provides access to both SWD pins and the [System Console](system_console.md).
This essentially moves the solder pads from beside the [Pixhawk Debug Mini](#pixhawk-debug-mini) into the connector, and also adds an SWO pin.

This port is specified for use in FMUv5x, FMUv6, FMUv6x.

The pinout is as shown below (pins required for debugging are bold):

|  핀 | 신호         |
| -: | :--------- |
|  1 | **VREF**   |
|  2 | 콘솔 TX      |
|  3 | 콘솔 RX      |
|  4 | **SWDIO**  |
|  5 | **SWDCLK** |
|  6 | SWO        |
|  7 | GPIO1      |
|  8 | GPIO2      |
|  9 | nRST       |
| 10 | **GND**    |

The GPIO1/2 pins are free pins that can be used to generate signals in software for timing analysis with a logic analyzer.

The socket is a _10-pin JST SH_ - Digikey number: [BM10B-SRSS-TB(LF)(SN)](https://www.digikey.com/en/products/detail/jst-sales-america-inc/BM10B-SRSS-TB/926702) (vertical mount) or [SM10B-SRSS-TB(LF)(SN)](https://www.digikey.com/en/products/detail/jst-sales-america-inc/BM10B-SRSS-TB/926702) (side mount).

You can connect to the debug port using a [cable like this one](https://www.digikey.com/products/en?keywords=A10SR10SR30K203A).

<!-- FIXME: better to have image showing proper connections for SWD+SWO -->

![10-pin JST SH Cable](../../assets/debug/cable_10pin_jst_sh.jpg)

## Debug Probes for PX4 Hardware {#debug-probes}

Flight controllers commonly provide a [single debug port](#autopilot-debug-ports) that exposes both the [SWD Interface](#debug-signals) and [System Console](system_console.md).

There are several debug probes that are tested and supported for connecting to one or both of these interfaces:

- [SEGGER J-Link](../debug/probe_jlink.md): commercial probe, no built-in serial console, requires adapter.
- [Black Magic Probe](../debug/probe_bmp.md): integrated GDB server and serial console, requires adapter.
- [STLink](../debug/probe_stlink.md): best value, integrated serial console, adapter must be soldered.
- [MCU-Link](../debug/probe_mculink.md): best value, integrated serial console, requires adapter.

An adapter to connect to the debug port may come with your flight controller or debug probe.
Other options are given below.

## Debug Adapters

### Holybro Pixhawk Debug Adapter

The [Holybro Pixhawk Debug Adapter](https://holybro.com/products/pixhawk-debug-adapter) is _highly recommended_ when debugging controllers that use one of the Pixhawk-standard debug connectors.

It is the easiest way to connect:

- Flight controllers that use either the [Pixhawk Debug Full](#pixhawk-debug-full) (10-pin SH) or [Pixhawk Debug Mini](#pixhawk-debug-mini) (6-pin SH) debug port.
- SWD debug probes that support the 10-pin ARM compatible interface standard used by the [Segger JLink EDU mini](../debug/probe_jlink.md) or 20-pin compatible with the Segger JLink or STLink.

![Holybro Pixhawk Debug Adapter](../../assets/debug/holybro_pixhawk_debug_adapter.png)

### CUAV C-ADB Pixhawk Debug Adapter

The [CUAV C-ADB Secondary Development Pixhawk Flight Controller Debug Adapter](https://store.cuav.net/shop/cuav-c-adb/) comes with an [STLinkv3-MINIE Debug Probe](../debug/probe_stlink.md).

This has a ports for connecting to the [Pixhawk Debug Full](#pixhawk-debug-full) (10-pin SH) and CUAV-standard DSU interface (but not the [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) (6-pin SH)).

The M2 connector on the adaptor is 14-pin CN4 STDC14 (see the [STLinkv3-MINIE User Manual](https://www.st.com/resource/en/user_manual/um2910-stlinkv3minie-debuggerprogrammer-tiny-probe-for-stm32-microcontrollers-stmicroelectronics.pdf) for more information).
The cable used to connect the M2 and the STLinkv3-MINIE comes with the adaptor.

![CUAV C-ADB adaptor connected to the STLinkv3-MINIE](../../assets/debug/cuav_c-adb_debug_adapter/hero.jpg)

### Debug Probe Adapters

Some SWD [debug probes](#debug-probes) come with adapters/cables for connecting to common Pixhawk [debug ports](#debug-ports).
Probes that are known to come with connectors are listed below:

- [Zubax BugFace BF1](../debug/probe_bmp.md#dronecode-probe): comes with a connector for attaching to the [Pixhawk Debug Mini](#pixhawk-debug-mini)

### Board-specific Adapters

Some manufacturers provide cables to make it easy to connect the SWD interface and [System Console](../debug/system_console.md).

- [CUAV V5nano](../flight_controller/cuav_v5_nano.md#debug_port) and [CUAV V5+](../flight_controller/cuav_v5_plus.md#debug-port) include this debug cable:

![6-pin JST SH Cable](../../assets/debug/cuav_v5_debug_cable.jpg)

### Custom Cables

You can also create custom cables for connecting to different boards or probes:

- Connect `SWDIO`, `SWCLK` and `GND` pins on the debug probe to the corresponding pins on the debug port.
- Connect the VREF pin, if supported by the debug probe.
- Connect the remaining pins, if present.

See the [STLinkv3-MINIE](probe_stlink.md) for a guide on how to solder a custom cable.

:::tip
Where possible, we highly recommend that you create or obtain an adapter board rather than custom cables for connecting to SWD/JTAG debuggers and computers.
This reduces the risk or poor wiring contributing to debugging problems, and has the benefit that adapters usually provide a common interface for connecting to multiple popular flight controller boards.
:::

<!-- Reference links used above -->

[swd]: https://developer.arm.com/documentation/ihi0031/a/The-Serial-Wire-Debug-Port--SW-DP-
[itm]: https://developer.arm.com/documentation/ddi0403/d/Appendices/Debug-ITM-and-DWT-Packet-Protocol?lang=en
[etm]: https://developer.arm.com/documentation/ihi0064/latest/
