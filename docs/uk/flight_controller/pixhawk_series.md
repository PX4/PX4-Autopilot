# Pixhawk Series

[Pixhawk<sup>&reg;</sup>](https://pixhawk.org/) is an independent open-hardware project providing readily-available, low-cost, and high-end, _autopilot hardware designs_ to the academic, hobby and industrial communities.

Pixhawk is the reference hardware platform for PX4, and runs PX4 on the [NuttX](https://nuttx.apache.org/) OS.

Виробники створили багато різних плат на основі відкритих дизайнів, з форм-факторами, які оптимізовані для застосувань від перевезення вантажів до гоночних змагань від першої особи (FPV).

:::tip
For computationally intensive tasks (e.g. computer vision) you will need a separate companion computer (e.g. [Raspberry Pi 2/3 Navio2](../flight_controller/raspberry_pi_navio2.md)) or a platform with an integrated companion solution.
:::

## Ключові переваги

Key benefits of using a _Pixhawk series_ controller include:

- Підтримка програмного забезпечення - як офіційне апаратне забезпечення PX4, це наші найкраще підтримувані плати.
- Гнучкість у відношенні до апаратного забезпечення, яке можна під'єднати.
- Висока якість.
- Висококастомізований у плані форм-фактору.
- Широко використовується і, отже, добре протестований/стабільний.
- Automated update of latest firmware via _QGroundControl_ (end-user friendly).

## Підтримувані плати

The PX4 Project uses [Pixhawk Standard Autopilots](../flight_controller/autopilot_pixhawk_standard.md) as reference hardware.
Це контролери, які повністю сумісні зі стандартом Pixhawk (включаючи використання товарних знаків) і які все ще виробляються.

:::info
The PX4 maintenance and test teams maintain and support these standard boards.
:::

Pixhawk-like boards that are not fully compliant with the specification may be [manufacturer-supported](../flight_controller/autopilot_manufacturer_supported.md), [experimental/discontinued](../flight_controller/autopilot_experimental.md), or unsupported.

Решта цієї теми пояснює трохи більше про серію Pixhawk, але її не обов'язково читати.

## Background

The [Pixhawk project](https://pixhawk.org/) creates open hardware designs in the form of schematics, which define a set of components (CPU, sensors, etc.) and their connections/pin mappings.

Manufacturers are encouraged to take the [open designs](https://github.com/pixhawk/Hardware) and create products that are best suited to a particular market or use case (the physical layout/form factor not part of the open specification). Плати на основі того ж дизайну сумісні за принципом двійкової сумісності.

:::info
While a physical connector standard is not mandated, newer products generally follow the [Pixhawk Connector Standard](https://pixhawk.org/pixhawk-connector-standard/).
:::

The project also creates reference autopilot boards based on the open designs, and shares them under the same [licence](#licensing-and-trademarks).

<a id="fmu_versions"></a>

### Версії FMU

Проєкт Pixhawk створив безліч різних відкритих дизайнів/схем.
Всі плати, які базуються на дизайні, мають бути бінарно сумісними (запускати ту саму прошивку).

Кожен дизайн має назву за допомогою позначення: FMUvX (наприклад: FMUv1, FMUv2, FMUv3, FMUv4 тощо).
Вищі номери FMU вказують на те, що плата є більш сучасною, але це може не означати збільшення функціональності (версії можуть бути майже ідентичними - відрізняються лише за підключенням проводів).

PX4 _users_ generally do not need to know very much about FMU versions:

- _QGroundControl_ automatically downloads the correct firmware for a connected autopilot (based on its FMU version "under the hood").
- Вибір контролера зазвичай ґрунтується на фізичних обмеженнях/форм-факторі, а не на версії FMU.

:::info
The exception is that if you're using FMUv2 firmware it is [limited to 1MB of flash](../flight_controller/silicon_errata.md#fmuv2-pixhawk-silicon-errata).
Щоб вмістити PX4 в цей обмежений простір, багато модулів за замовчуванням вимкнені.
You may find that some [parameters are missing](../advanced_config/parameters.md#missing) and that some hardware does not work "out of the box".
:::

PX4 _developers_ need to know the FMU version of their board, as this is required to build custom hardware.

На дуже високому рівні, основні відмінності полягають у наступному:

- **FMUv2:** Single board with STM32427VI processor ([Pixhawk 1 (Discontinued)](../flight_controller/pixhawk.md), [pix32](../flight_controller/holybro_pix32.md), [Pixfalcon](../flight_controller/pixfalcon.md), [Drotek DroPix](../flight_controller/dropix.md))
- **FMUv3:** Identical to FMUv2, but usable flash doubled to 2MB ([Hex Cube Black](../flight_controller/pixhawk-2.md),[CUAV Pixhack v3](../flight_controller/pixhack_v3.md),[mRo Pixhawk](../flight_controller/mro_pixhawk.md), [Pixhawk Mini (Discontinued)](../flight_controller/pixhawk_mini.md))
- **FMUv4:** Increased RAM. Швидший процесор. Більше послідовних портів. No IO processor ([Pixracer](../flight_controller/pixracer.md))
- **FMUv4-PRO:** Slightly increased RAM. Більше послідовних портів. IO processor ([Pixhawk 3 Pro](../flight_controller/pixhawk3_pro.md))
- **FMUv5:** New processor (F7).
  Набагато швидший.
  Більше RAM.
  Більше CAN шин.
  Набагато більш налаштовуваний.
  ([Pixhawk 4](../flight_controller/pixhawk4.md),[CUAV v5](../flight_controller/cuav_v5.md),[CUAV V5+](../flight_controller/cuav_v5_plus.md),[CUAV V5 nano](../flight_controller/cuav_v5_nano.md))
- **FMUv5X:** New processor (F7).
  Набагато швидший, модульний дизайн.
  Більш надійний.
  Більше резервування.
  Більше RAM (1MB).
  Більше CAN шин.
  Much more configurable & customizable.
  ([Pixhawk 5X](../flight_controller/pixhawk5x.md), Skynode)
- **FMUv6C:**
  ([Holybro Pixhawk 6C Mini](../flight_controller/pixhawk6c_mini.md), [Holybro Pixhawk 6C](../flight_controller/pixhawk6c.md))
- **FMUv6X:**
  ([CUAV Pixhawk V6X](../flight_controller/cuav_pixhawk_v6x.md),[Holybro Pixhawk 6X](../flight_controller/pixhawk6x.md))
- **FMUv6X-RT:** Faster MCU core (1GHz) (vs 480Mhz on 6X).
  Більше RAM (2Mb).
  Більше флеш пам'яті (64 Мб) (2 Мб на v6X/v5X).
  ([Holybro Pixhawk 6X-RT](../flight_controller/pixhawk6x-rt.md))

<a id="licensing-and-trademarks"></a>

### Ліцензування та товарні знаки

Pixhawk project schematics and reference designs are licensed under [CC BY-SA 3](https://creativecommons.org/licenses/by-sa/3.0/legalcode).

The license allows you to use, sell, share, modify and build on the files in almost any way you like - provided that you give credit/attribution, and that you share any changes that you make under the same open source license (see the [human readable version of the license](https://creativecommons.org/licenses/by-sa/3.0/) for a concise summary of the rights and obligations).

:::info
Boards that are _derived directly_ from Pixhawk project schematic files (or reference boards) must be open sourced.
Вони не можуть бути комерційно ліцензовані як власні продукти.
:::

Manufacturers can create (compatible) _fully independent products_ by first generating fresh schematic files that have the same pin mapping/components as the FMU designs.
Продукти, що базуються на незалежно створених схемах, вважаються оригінальними роботами і можуть бути ліцензовані за потреби.

Назви продуктів / бренди також можуть бути торговими марками.
Торгові назви не можуть бути використані без дозволу власника.

:::tip
_Pixhawk_ is a trademark, and cannot be used in product names without permission.
:::

## Додаткова інформація

### Світлодіоди

All _Pixhawk-series_ flight controllers support:

- A user facing RGB _UI LED_ to indicate the current _readiness to fly_ status of the vehicle. Зазвичай це дуже яскравий I2C, який може бути встановлений або не встановлений на платі (наприклад FMUv4 не має такого на борту, і зазвичай використовує світлодіод, встановлений на GPS).
- Three _Status LED_s that provide lower level power status, bootloader mode and activity, and error information.

To interpret the LEDs see: [LED Meanings](../getting_started/led_meanings.md).
