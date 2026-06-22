# Порт для налагодження SWD

PX4 runs on ARM Cortex-M microcontrollers, which contain dedicated hardware for interactive debugging via the [_Serial Wire Debug (SWD)_][swd] interface and non-invasive profiling and high-bandwidth tracing via the [_Serial Wire Output (SWO)_][itm] and [_TRACE_ pins][etm].

Інтерфейс відладки SWD дозволяє прямий, низькорівневий, апаратний доступ до процесора мікроконтролера та периферійних пристроїв, тому він не залежить від будь-якого програмного забезпечення на пристрої.
Отже, його можна використовувати для налагодження завантажувальних програм та операційних систем, таких як NuttX.

## Debug Signals {#debug-signals}

Чотири сигнали необхідні для відлагодження (в жирному шрифті), а решту лише рекомендується.

| Назва                                                           | Тип    | Опис                                                                                                         |
| :-------------------------------------------------------------- | :----- | :----------------------------------------------------------------------------------------------------------- |
| **GND**                                                         | Power  | Спільний потенціал, спільна основа.                                                          |
| **VREF**                                                        | Power  | Цільове довідкове напруга дозволяє засоби налагодження використовувати рівнеміри на сигнали. |
| **SWDIO**                                                       | I/O    | Пін даних для послідовного знавантаження через мережу.                                       |
| **SWCLK**                                                       | Вхід   | Пін годинника для послідовного знавантаження через мережу.                                   |
| nRST                                                            | Вхід   | PIN скидання є необов’язковим (n = активним низьким).                     |
| SWO                                                             | Output | Однопровідний шлейф асинхронних даних з можливістю виведення даних ITM та DWT.               |
| TRACECK                                                         | Output | Трасування годинника для паралельної шини.                                                   |
| TRACED[0-3] | Output | Трасування синхронної шини даних з 1, 2 чи 4 бітами.                                         |

Пін скидання апаратного забезпечення є необов'язковим, оскільки більшість пристроїв також можуть бути скинуті через лінії SWD. Однак швидке скидання пристрою за допомогою кнопки може бути великим плюсом для розробки.

SWO-пін може випромінювати дані профілювання в реальному часі з наносекундним відмітками часу, тому настійно рекомендується мати доступ до нього для відлагодження.

Піни TRACE потребують спеціалізованих засобів відлагодження для роботи з високою пропускною здатністю та наступним декодуванням потоку даних.
Зазвичай вони недоступні і зазвичай використовуються лише для відлагодження дуже конкретних питань з часом.

## Autopilot Debug Ports {#debug-ports}

Flight controllers commonly provide a single debug port that exposes both the [SWD Interface](#debug-signals) and [System Console](system_console.md).

The [Pixhawk Connector Standards](#pixhawk-standard-debug-ports) formalize the port that must be used in each FMU version.
However there are still many boards that use different pinouts or connectors, so we recommend you check the [documentation for your autopilot](../flight_controller/index.md) to confirm port location and pinout.

Місцезнаходження порту налагодження та роз'єми для підмножини автопілотів зв'язані нижче:

<a id="port-information"></a>

| Автопілот                                                                                               | Відладочний порт                                                         |
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

Проект Pixhawk визначив стандартну схему виводів та тип роз'єму для різних випусків Pixhawk FMU:

:::tip
Check your [specific board](#port-information) to confirm the port used.
:::

| Версія FMU | Версія Pixhawk      | Відладочний порт                          |
| :--------- | :------------------ | :---------------------------------------- |
| FMUv2      | Pixhawk / Pixhawk 1 | 10 pin ARM Debug                          |
| FMUv3      | Pixhawk 2           | 6 pin SUR Debug                           |
| FMUv4      | Pixhawk 3           | [Pixhawk Debug Mini](#pixhawk-debug-mini) |
| FMUv5      | Pixhawk 4 FMUv5     | [Pixhawk Debug Mini](#pixhawk-debug-mini) |
| FMUv5X     | Pixhawk 5X          | [Pixhawk Debug Full](#pixhawk-debug-full) |
| FMUv6      | Pixhawk 6           | [Pixhawk Debug Full](#pixhawk-debug-full) |
| FMUv6X     | Pixhawk 6X          | [Pixhawk Debug Full](#pixhawk-debug-full) |
| FMUv6X-RT  | Pixhawk 6X-RT       | [Pixhawk Debug Full](#pixhawk-debug-full) |

:::info
There FMU and Pixhawk versions are (only) consistent after FMUv5X.
:::

### Pixhawk Debug Mini

The [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) defines the _Pixhawk Debug Mini_, a _6-Pin SH Debug Port_ that provides access to both SWD pins and the [System Console](system_console.md).

Це використовується в FMUv4 та FMUv5.

Схема виводів показана нижче (виводи, необхідні для налагодження, виділені жирним шрифтом):

| Pin | Сигнал     |
| --: | :--------- |
|   1 | **VREF**   |
|   2 | Console TX |
|   3 | Console RX |
|   4 | **SWDIO**  |
|   5 | **SWDCLK** |
|   6 | **GND**    |

Визначення порту налагодження містить наступні припояні пластины (на платі поряд із роз'ємом):

| Pad | Сигнал | Напруга               |
| --: | :----- | :-------------------- |
|   1 | nRST   | +3.3V |
|   2 | GPIO1  | +3.3V |
|   3 | GPIO2  | +3.3V |

The socket is a _6-pin JST SH_ - Digikey number: [BM06B-SRSS-TBT(LF)(SN)](https://www.digikey.com/en/products/detail/jst-sales-america-inc/BM06B-SRSS-TBT/1785724) (vertical mount), [SM06B-SRSS-TBT(LF)(SN)](https://www.digikey.com/en/products/detail/jst-sales-america-inc/SM06B-SRSS-TB/926712) (side mount).

You can connect to the debug port using a [cable like this one](https://www.digikey.com/en/products/detail/jst-sales-america-inc/A06SR06SR30K152A/6009379).

![6-pin JST SH Cable](../../assets/debug/cable_6pin_jst_sh.jpg)

### Порти відладки Pixhawk Full

The [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) defines _Pixhawk Debug Full_, a _10-Pin SH Debug Port_ that provides access to both SWD pins and the [System Console](system_console.md).
This essentially moves the solder pads from beside the [Pixhawk Debug Mini](#pixhawk-debug-mini) into the connector, and also adds an SWO pin.

Цей порт вказаний для використання в FMUv5x, FMUv6, FMUv6x.

Схема виводів показана нижче (виводи, необхідні для налагодження, виділені жирним шрифтом):

| Pin | Сигнал     |
| --: | :--------- |
|   1 | **VREF**   |
|   2 | Console TX |
|   3 | Console RX |
|   4 | **SWDIO**  |
|   5 | **SWDCLK** |
|   6 | SWO        |
|   7 | GPIO1      |
|   8 | GPIO2      |
|   9 | nRST       |
|  10 | **GND**    |

Піни GPIO1/2 є вільними пінами, які можуть бути використані для генерації сигналів у програмному забезпеченні для аналізу часу з логічним аналізатором.

The socket is a _10-pin JST SH_ - Digikey number: [BM10B-SRSS-TB(LF)(SN)](https://www.digikey.com/en/products/detail/jst-sales-america-inc/BM10B-SRSS-TB/926702) (vertical mount) or [SM10B-SRSS-TB(LF)(SN)](https://www.digikey.com/en/products/detail/jst-sales-america-inc/BM10B-SRSS-TB/926702) (side mount).

You can connect to the debug port using a [cable like this one](https://www.digikey.com/products/en?keywords=A10SR10SR30K203A).

<!-- FIXME: better to have image showing proper connections for SWD+SWO -->

![10-pin JST SH Cable](../../assets/debug/cable_10pin_jst_sh.jpg)

## Debug Probes for PX4 Hardware {#debug-probes}

Flight controllers commonly provide a [single debug port](#autopilot-debug-ports) that exposes both the [SWD Interface](#debug-signals) and [System Console](system_console.md).

Є кілька зондів відлагодження, які були перевірені та підтримуються для підключення до одного або обох цих інтерфейсів:

- [SEGGER J-Link](../debug/probe_jlink.md): commercial probe, no built-in serial console, requires adapter.
- [Black Magic Probe](../debug/probe_bmp.md): integrated GDB server and serial console, requires adapter.
- [STLink](../debug/probe_stlink.md): best value, integrated serial console, adapter must be soldered.
- [MCU-Link](../debug/probe_mculink.md): best value, integrated serial console, requires adapter.

Адаптер для підключення до роз'єму відладки може поставлятися разом із вашим контролером польоту або відлагоджувальним зондом.
Інші варіанти наведено нижче.

## Адаптери для відлагоджування

### Адаптер відлагодження Holybro Pixhawk

The [Holybro Pixhawk Debug Adapter](https://holybro.com/products/pixhawk-debug-adapter) is _highly recommended_ when debugging controllers that use one of the Pixhawk-standard debug connectors.

Це найлегший спосіб підключення:

- Flight controllers that use either the [Pixhawk Debug Full](#pixhawk-debug-full) (10-pin SH) or [Pixhawk Debug Mini](#pixhawk-debug-mini) (6-pin SH) debug port.
- SWD debug probes that support the 10-pin ARM compatible interface standard used by the [Segger JLink EDU mini](../debug/probe_jlink.md) or 20-pin compatible with the Segger JLink or STLink.

![Holybro Pixhawk Debug Adapter](../../assets/debug/holybro_pixhawk_debug_adapter.png)

### Адаптер відлагодження CUAV C-ADB Pixhawk

The [CUAV C-ADB Secondary Development Pixhawk Flight Controller Debug Adapter](https://store.cuav.net/shop/cuav-c-adb/) comes with an [STLinkv3-MINIE Debug Probe](../debug/probe_stlink.md).

This has a ports for connecting to the [Pixhawk Debug Full](#pixhawk-debug-full) (10-pin SH) and CUAV-standard DSU interface (but not the [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) (6-pin SH)).

The M2 connector on the adaptor is 14-pin CN4 STDC14 (see the [STLinkv3-MINIE User Manual](https://www.st.com/resource/en/user_manual/um2910-stlinkv3minie-debuggerprogrammer-tiny-probe-for-stm32-microcontrollers-stmicroelectronics.pdf) for more information).
Кабель, який використовується для підключення M2 та STLinkv3-MINIE, постачається з адаптером.

![CUAV C-ADB adaptor connected to the STLinkv3-MINIE](../../assets/debug/cuav_c-adb_debug_adapter/hero.jpg)

### Адаптери для відлагоджування

Some SWD [debug probes](#debug-probes) come with adapters/cables for connecting to common Pixhawk [debug ports](#debug-ports).
Зонди, про які відомо, що поставляються з роз'ємами, перераховані нижче:

- [Zubax BugFace BF1](../debug/probe_bmp.md#dronecode-probe): comes with a connector for attaching to the [Pixhawk Debug Mini](#pixhawk-debug-mini)

### Адаптери, специфічні для плати

Some manufacturers provide cables to make it easy to connect the SWD interface and [System Console](../debug/system_console.md).

- [CUAV V5nano](../flight_controller/cuav_v5_nano.md#debug_port) and [CUAV V5+](../flight_controller/cuav_v5_plus.md#debug-port) include this debug cable:

![6-pin JST SH Cable](../../assets/debug/cuav_v5_debug_cable.jpg)

### Користувацькі кабелі

Ви також можете створити власні кабелі для підключення до різних плат або зондів:

- Connect `SWDIO`, `SWCLK` and `GND` pins on the debug probe to the corresponding pins on the debug port.
- Підключіть контакт VREF, якщо його підтримує засіб відлагодження.
- Підключіть залишкові контакти, якщо вони є.

See the [STLinkv3-MINIE](probe_stlink.md) for a guide on how to solder a custom cable.

:::tip
Where possible, we highly recommend that you create or obtain an adapter board rather than custom cables for connecting to SWD/JTAG debuggers and computers.
Це зменшує ризик неправильного підключення проводів, що призводить до проблем з налагодженням, і має перевагу в тому, що адаптери зазвичай надають спільний інтерфейс для підключення до кількох популярних плат керування польотом.
:::

<!-- Reference links used above -->

[swd]: https://developer.arm.com/documentation/ihi0031/a/The-Serial-Wire-Debug-Port--SW-DP-
[itm]: https://developer.arm.com/documentation/ddi0403/d/Appendices/Debug-ITM-and-DWT-Packet-Protocol?lang=en
[etm]: https://developer.arm.com/documentation/ihi0064/latest/
