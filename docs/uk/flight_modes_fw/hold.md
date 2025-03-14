# Режим утримання (з нерухомим крилом)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Hold_ flight mode causes the vehicle to loiter (circle) around its current GPS position and maintain its current altitude.

:::tip
_Hold mode_ can be used to pause a mission or to help you regain control of a vehicle in an emergency.
Зазвичай він активується за допомогою наперед заданого перемикача.
:::

::: info

- Режим автоматичний - для керування апаратом не потрібно втручання користувача.
- Режим вимагає глобальної оцінки 3D-позиції (з GPS або виведеної з [локальної позиції](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Літаючі транспортні засоби не можуть переключатися на цей режим без глобального положення.
  - Літаючі транспортні засоби перейдуть в режим аварійної безпеки, якщо втратять оцінку положення.
  - Роззброєні транспортні засоби можуть переключатися в режим без дійсної оцінки позиції, але не можуть озброюватися.
- Режим вимагає, щоб швидкість вітру та час польоту були в межах допустимих значень (вказано через параметри).
- Перемикачі керування RC можуть використовуватися для зміни режимів польоту на будь-якому транспортному засобі.
- Рух стіків радіокерування ігнорується.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Технічний підсумок

Літак кружляє навколо позиції утримання GPS на поточній висоті.
The vehicle will first ascend to [NAV_MIN_LTR_ALT](#NAV_MIN_LTR_ALT) if the mode is engaged below this altitude.

Рух стіків радіокерування ігнорується.

### Параметри

Поведінку режиму утримання можна налаштувати за допомогою наведених нижче параметрів.

| Параметр                                                                                                                                                                | Опис                                                                                                                                                             |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_LOITER_RAD)                                                    | Радіус кола обертання.                                                                                                                           |
| <a id="NAV_MIN_LTR_ALT"></a>[NAV_MIN_LTR_ALT](../advanced_config/parameter_reference.md#NAV_MIN_LTR_ALT) | Мінімальна висота для режиму очікування (транспортний засіб підніметься на цю висоту, якщо режим увімкнуто на меншій висоті). |

## Дивіться також

[Hold Mode (MC)](../flight_modes_mc/hold.md)

<!-- this maps to AUTO_LOITER in flight mode state machine -->
