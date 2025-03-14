# Initial Setup & Configuration

Ми рекомендуємо розробникам отримати базове обладнання та програмне забезпечення, описане нижче (або подібне).

## Базове обладнання

:::tip
PX4 can be used with a much wider range of equipment than described here, but new developers will benefit from going with one of the standard setups.
A Taranis RC and a mid-range Android tablet make a very inexpensive field kit.
:::

Нижченаведене обладнання дуже рекомендується:

- **RC controller** for the safety pilot
  - [Taranis Plus](https://www.frsky-rc.com/product/taranis-x9d-plus-2/) RC control (or equivalent)

- **Development computer**

  ::: info
  The listed computers have acceptable performance, but a more recent and powerful computer is recommended.

:::

  - Lenovo Thinkpad with i5-core running Windows 11
  - MacBook Pro (early 2015 and later) with macOS 10.15 or later
  - Lenovo Thinkpad i5 with Ubuntu Linux 20.04 or later

- **Ground control station** (computer or tablet):
  - iPad (may require Wifi telemetry adapter)
  - Будь-який ноутбук MacBook або Ubuntu Linux (може бути комп'ютером для розробки)
  - A recent mid-range Android tablet or phone with a large enough screen to run _QGroundControl_ effectively (6 inches).

- **Vehicle capable of running PX4**:
  - [Get a prebuilt vehicle](../complete_vehicles_mc/index.md)
  - [Build your own](../frames_multicopter/kits.md)

- **Safety glasses**

- **Tether** (multicopter only - for more risky tests)

## Конфігурація рухомого засобу

Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md) for a **desktop OS**.

Для налаштування засобу:

1. [Install PX4 firmware](../config/firmware.md#installing-px4-main-beta-or-custom-firmware) (including "custom" firmware with your own changes).
2. [Start with the airframe](../config/airframe.md) that best-matches your vehicle from the [airframe reference](../airframes/airframe_reference.md).
3. [Basic Configuration](../config/index.md) explains how to perform basic configuration.
4. [Parameter Configuration](../advanced_config/parameters.md) explains how you can find and modify individual parameters.

::: info

- _QGroundControl_ mobile variants do not support vehicle configuration.
- The _daily build_ includes development tools and new features that are not available in the official release.
- Конфігурації в довіднику планерів літали на реальних засобах та є хорошою стартовою точкою для "відриву від землі".

:::
