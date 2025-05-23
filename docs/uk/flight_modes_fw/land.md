# Режим посадки (фіксоване крило)

<img src="../../assets/site/position_fixed.svg" title="Position estimate required (e.g. GPS)" width="30px" />

The _Land_ flight mode causes the vehicle to descend at the position where the mode was engaged, following a circular path until touchdown.
Після посадки транспортний засіб вимкнеться через короткий проміжок часу (за замовчуванням).

:::warning
Fixed-wing _land mode_ should only be used in an **emergency**!
Транспортний засіб опуститься навколо поточного місцезнаходження незалежно від придатності підлягаючої території, і сяде на землю, слідуючи круговому шляху польоту.

Where possible, instead use [Return mode](../flight_modes_fw/return.md) with a predefined [Fixed-wing mission landing](../flight_modes_fw/mission.md#mission-landing).
:::

::: info

- Режим автоматичний - для керування апаратом не потрібно втручання користувача.
- Режим потребує принаймні дійсної локальної оцінки позиції (не потребує глобальної позиції).
  - Літаючі транспортні засоби не можуть переключатися на цей режим без глобального положення.
  - Літаючі транспортні засоби перейдуть в режим аварійної безпеки, якщо втратять оцінку положення.
- Режим перешкоджає зброюванню (транспортний засіб повинен бути зброєний при переході на цей режим).
- Перемикачі керування RC можуть використовуватися для зміни режимів польоту на будь-якому транспортному засобі.
- Рух стіків радіокерування ігнорується.
- The mode can be triggered using the [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) MAVLink command, or by explicitly switching to Land mode.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Технічний підсумок

Режим посадки змушує літальний засіб прямувати по спускаючій круговій траєкторії (шнуру) до посадки.

When the mode is engaged, the vehicle starts to loiter around the current vehicle position with loiter radius [NAV_LOITER_RAD](#NAV_LOITER_RAD) and begins to descend with a constant descent speed.
The descent speed is calculated using [FW_LND_ANG](#FW_LND_ANG) and the set landing airspeed [FW_LND_AIRSPD](#FW_LND_AIRSPD).
The vehicle will flare if configured to do so (see [Flaring](../flight_modes_fw/mission.md#flaring-roll-out)), and otherwise proceed circling with the constant descent rate until landing is detected.

[Manual nudging](../flight_modes_fw/mission.md#automatic-abort) and [automatic land abort](../flight_modes_fw/mission.md#nudging) are not available in land mode.

### Параметри

Поведінку режиму приземлення можна налаштувати за допомогою наведених нижче параметрів.

| Параметр                                                                                                                                        | Опис                                                                                            |
| ----------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| <a id="NAV_LOITER_RAD"></a>[NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_LOITER_RAD) | Радіус блукання, який контролер відстежує протягом усієї послідовності посадки. |
| <a id="FW_LND_ANG"></a>[FW_LND_ANG](../advanced_config/parameter_reference.md#FW_LND_ANG)             | Виставте кут шляху пункту налаштувань.                                          |
| <a id="FW_LND_AIRSPD"></a>[FW_LND_AIRSPD](../advanced_config/parameter_reference.md#FW_LND_AIRSPD)    | Налаштування швидкості.                                                         |

## Дивіться також

- [Режим посадки (MC)](../flight_modes_mc/land.md)
- [Land Mode (VTOL)](../flight_modes_vtol/land.md)
