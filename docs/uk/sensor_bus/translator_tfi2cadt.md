# TFI2CADT01 - Перетворювач адреси I²C

[TFI2CADT01](https://github.com/ThunderFly-aerospace/TFI2CADT01) is an address translator module that is compatible with Pixhawk and PX4.

Перетворення адрес дозволяє кільком пристроям I2C з однаковою адресою співіснувати в мережі I2C.
Модуль може знадобитися, якщо використовуються кілька пристроїв, які мають однакову заводську адресу.

Модуль має вхідну та вихідну сторони.
Датчик підключається до головного пристрою на одній стороні.
На вихідній стороні можуть бути підключені датчики, адреси яких потрібно перетворити.
Модуль містить дві пари конекторів, кожна пара відповідає за різні перетворення.

![TFI2CADT - i2c address translator](../../assets/peripherals/i2c_tfi2cadt/tfi2cadt01a_both_sides.jpg)

:::info
[TFI2CADT01](https://github.com/ThunderFly-aerospace/TFI2CADT01) is designed as open-source hardware with GPLv3 license.
It is commercially available from [ThunderFly](https://www.thunderfly.cz/) company or from [Tindie eshop](https://www.tindie.com/products/thunderfly/tfi2cadt01-i2c-address-translator/).
:::

## Метод перетворення адрес

TFI2CADT01 виконує операцію XOR на викликаній адресі.
Таким чином, нову адресу пристрою можна знайти, взявши оригінальну адресу та застосувавши операцію XOR з значенням, вказаним на модулі.
За замовчуванням, вихід 1 виконує XOR зі значенням 0x08, а другий порт - з 0x78.
Коротко заданої каси можна змінити значення XOR на 0x0f для першого та 0x7f для другого порту.

Якщо вам потрібне власне значення для перетворення адрес, зміна конфігураційних резисторів дозволяє встановити будь-яке значення XOR.

## Приклад використання

The tachometer sensor [TFRPM01](../sensor/thunderfly_tachometer.md) can be set to two different addresses using a solder jumper.
Якщо автопілот має три шини, тільки 6 датчиків можуть бути підключені і жодна шина не залишається вільною (2 доступні адреси \* 3 порти I2C).
У деяких мультикоптерах або рішеннях VTOL є необхідність вимірювати оберти хвилину RPM 8 або більше елементів.
The [TFI2CADT01](https://www.tindie.com/products/thunderfly/tfi2cadt01-i2c-address-translator/) is highly recommended in this case.

![Multiple sensors](../../assets/peripherals/i2c_tfi2cadt/tfi2cadt01_multi_tfrpm01.jpg)

Наступна схема показує, як підключити 6 TFRPM01 до однієї шини автопілота.
Додавши ще один TFI2CADT01, до тієї ж шини можна підключити ще 4 пристрої.

[![Connection of multiple sensors](https://mermaid.ink/img/pako:eNptkd9rwjAQx_-VcE8dtJB2ukEfBLEWfJCJy8CHvgRznQH7gzSBDfF_33VZB2oCyf3I576XcBc4dgohh08j-xMTRdUyWuX2I6LNErY7zJh0tuv1ubNP_7csSRZsudlHS22GHlGxAduhM3fEfrdNI1GS4emK8a85fwSyGyC9A0S5yVbrg_DZKfLtCxH9JsjhaU7VvI7pfK3_NCg_NXmO3pwl5uYt9D0yAXoWoFNP4yM9H-kspJ0FtF8CdObpURtiaNA0UisaymWsrsCesMEKcnIV1tKdbQVVeyXU9UpaXCttOwO5NQ5jGKf1_t0ep9gzhZY04sYnrz9BI4mU)](https://mermaid-js.github.io/mermaid-live-editor/edit#pako:eNptkd9rwjAQx_-VcE8dtJB2ukEfBLEWfJCJy8CHvgRznQH7gzSBDfF_33VZB2oCyf3I576XcBc4dgohh08j-xMTRdUyWuX2I6LNErY7zJh0tuv1ubNP_7csSRZsudlHS22GHlGxAduhM3fEfrdNI1GS4emK8a85fwSyGyC9A0S5yVbrg_DZKfLtCxH9JsjhaU7VvI7pfK3_NCg_NXmO3pwl5uYt9D0yAXoWoFNP4yM9H-kspJ0FtF8CdObpURtiaNA0UisaymWsrsCesMEKcnIV1tKdbQVVeyXU9UpaXCttOwO5NQ5jGKf1_t0ep9gzhZY04sYnrz9BI4mU)

<!-- original mermaid graph
graph TD
    FMU(FMU - PX4 autopilot)
    FMU -- > AIR(Airspeed sensor)
    FMU -- > RPM1(TFRPM01C 0x50)
    FMU -- > RPM2(TFRPM01C 0x51)
    FMU -- > TFI2CEXT
    TFI2CEXT -- > ADT(TFI2CADT01: 0x0f, 0x7f)
    ADT -- > RPM3(Out1: TFRPM01C 0x50 - 0x5f)
    ADT -- > RPM4(Out1: TFRPM01C 0x51 - 0x5e)
    ADT -- > RPM5(Out2: TFRPM01C 0x50 - 0x2f)
    ADT -- > RPM6(Out2: TFRPM01C 0x52 - 0x2e)
-->

:::info
TFI2CADT01 does not contain any I2C buffer or accelerator.
As it adds additional capacitance on the bus, we advise combining it with some bus booster, e.g. [TFI2CEXT01](https://github.com/ThunderFly-aerospace/TFI2CEXT01).
:::

### Інші ресурси

- Datasheet of [LTC4317](https://www.analog.com/media/en/technical-documentation/data-sheets/4317fa.pdf)
