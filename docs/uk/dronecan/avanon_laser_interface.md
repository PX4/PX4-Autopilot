# Інтерфейс лазерного альтиметра Avionics Anonymous DroneCan

:::info
In 2022, UAVCAN (v0) was forked and is maintained as `DroneCAN`.
Хоча цей продукт все ще згадує "UAVCAN", він повністю сумісний з підтримкою DroneCAN в PX4.
:::

The [Avionics Anonymous Laser Altimeter Interface](https://www.tindie.com/products/avionicsanonymous/uavcan-laser-altimeter-interface/) allows a [number of common rangefinders](#supported_rangefinders) to be connected via the CAN bus (this is a more robust interface than I2C).

![Avionics Anonymous Laser Altimeter DroneCAN Interface](../../assets/hardware/sensors/avionics_anon_uavcan_alt_interface/avionics_anon_altimeter_uavcan_interface.jpg)

## Де купити

- [AvAnon Laser Interface](https://www.tindie.com/products/avionicsanonymous/uavcan-laser-altimeter-interface/)

<a id="supported_rangefinders"></a>

## Підтримувані дальномери

Повний список підтримуваних далекомірів можна знайти за посиланням вище.

Наступні далекоміри підтримуються на момент написання:

- Lightware SF30/D
- Lightware SF10/a
- Lightware SF10/b
- Lightware SF10/c
- Lightware SF11/c
- Lightware SF/LW20/b
- Lightware SF/LW20/c

## Налаштування програмного забезпечення

### Підключення

Далекомір (лазер) підключений до плати інтерфейсу AvAnon, яка підключена до одного з CAN-портів вашого автопілота.
Проводка відбувається згідно з виведенням контактів вище, або необхідні кабелі можна придбати, щоб підключити їх безпосередньо до вашої системи.
These are available at the links [here](https://www.tindie.com/products/avionicsanonymous/uavcan-laser-altimeter-interface/).

Інтерфейсна плата забезпечує фільтрований вихід живлення для лазера, але не забезпечує власного регулювання.
Отже, лазер повинен бути сумісним з напругою, яка подається на плату.

### Схема розташування виводів

### Конектор шини CAN

| Pin | Назва                         | Опис                                                                                                                                                             |
| --- | ----------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | POWER_IN | Джерело живлення. Підтримується 4.0-5.5В, але також повинно бути сумісним з підключеним лазером. |
| 2   | TX/SCL                        | TX для послідовного режиму, Clock для режиму I2C.                                                                                                |
| 3   | RX/SDA                        | RX для послідовного режиму, Data для режиму I2C.                                                                                                 |
| 4   | GND                           | Сигнальна / заземлювальна земля.                                                                                                                 |

### З'єднувач лазеру

| Pin | Назва                          | Опис                                                              |
| --- | ------------------------------ | ----------------------------------------------------------------- |
| 1   | POWER_OUT | Фільтрована потужність при напрузі живлення.      |
| 2   | CAN+                           | TX для послідовного режиму, Clock для режиму I2C. |
| 3   | RX/SDA                         | RX для послідовного режиму, Data для режиму I2C.  |
| 4   | GND                            | Сигнальна / заземлювальна земля.                  |

## Конфігурація PX4

To enable the laser altimeter you will need to [set the following parameters](../advanced_config/parameters.md) (in QGroundControl):

- Enable DroneCAN by setting [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) non zero.
- Enable DroneCAN rangefinder subscription by setting [UAVCAN_SUB_RNG](../advanced_config/parameter_reference.md#UAVCAN_SUB_RNG)
- Set the minimum and maximum range of the rangefinder using [UAVCAN_RNG_MIN](../advanced_config/parameter_reference.md#UAVCAN_RNG_MIN) and [UAVCAN_RNG_MAX](../advanced_config/parameter_reference.md#UAVCAN_RNG_MAX).
