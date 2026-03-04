# Holybro Kopis 2

[Holybro Kopis 2] (https://holybro.com/products/kopis2-hdv-free-shipping) - це готовий до польотів гоночний квадрокоптер для польотів FPV або в межах прямої видимості.

![Kopis 2](../../assets/hardware/holybro_kopis2.jpg)

## Де купити

_Kopis 2_ можна придбати від кількох виробників, включаючи:

- [Holybro](https://holybro.com/products/kopis2-hdv-free-shipping) <!-- item code 30069, 30070 -->
- [GetFPV](https://www.getfpv.com/holybro-kopis-2-6s-fpv-racing-drone-pnp.html)

Крім того, вам знадобиться:

- Передавач дистанційного керування. _Kopis 2_ може бути поставлений з приймачем FrSky або без приймача взагалі.
- Акумулятор LiPo та зарядний пристрій.
- FPV окуляри, якщо ви хочете літати в режимі FPV.
  Є багато сумісних варіантів, включаючи ці від [Fatshark](https://www.fatshark.com/product-page/dominator-v3).
  Ви також можете використовувати окуляри DJI FPV, якщо у вас є версія Kopis 2 з HDV.

  ::: інформація
  Підтримка FPV є абсолютно незалежною від PX4/контролера польоту.

:::

## Прошивка завантажувача PX4

_Kopis 2_ поставляється із заздалегідь встановленим Betaflight.

Перед завантаженням прошивки PX4 вам спочатку потрібно встановити завантажувач PX4.
Інструкції по встановленню завантажувача можна знайти в темі [Kakute F7](../flight_controller/kakutef7.md#bootloader) (це плата польотного контролера на _Kopis 2_).

:::tip
Ви завжди можете [перевстановити Betaflight](../advanced_config/bootloader_update_from_betaflight.md#reinstall-betaflight) пізніше, якщо захочете!
:::

## Встановлення/Налаштування

Після установки завантажувача, ви зможете під'єднати апарат до _QGroundControl_ через USB-кабель.

:::info
На момент написання статті _Kopis 2_ підтримується на QGroundControl _Daily Build_, а попередньо зібрана прошивка надається лише для master гілки (стабільні випуски ще не доступні).
:::

Для встановлення та налаштування PX4:

- [Завантажити Прошивку PX4](../config/firmware.md).
- [Встановіть Airframe](../config/airframe.md) в _Holybro Kopis 2_.
- Продовжуйте з [базовою конфігурацією](../config/index.md), включаючи калібрування датчиків та радіо.
