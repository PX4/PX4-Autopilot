# Режим Зависання або Hold (Мультикоптер)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Hold_ flight mode causes the vehicle to stop and hover at its current GPS position and altitude.

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
- Рух палиць дистанційного керування буде [за замовчуванням](#COM_RC_OVERRIDE) змінювати транспортний засіб на [режим позиції](../flight_modes_mc/position.md), якщо не виникне критична аварія батареї.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Технічний підсумок

Транспортний засіб ширяє у поточному положенні на поточній висоті.
The vehicle will first ascend to [NAV_MIN_LTR_ALT](#NAV_MIN_LTR_ALT) if the mode is engaged below this altitude.

RC stick movement will change the vehicle to [Position mode](../flight_modes_mc/position.md) (by [default](#COM_RC_OVERRIDE)).

### Параметри

Поведінку режиму утримання можна налаштувати за допомогою наведених нижче параметрів.

| Параметр                                                                                                                                                                | Опис                                                                                                                                                                                                                                                                                                                                     |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="NAV_MIN_LTR_ALT"></a>[NAV_MIN_LTR_ALT](../advanced_config/parameter_reference.md#NAV_MIN_LTR_ALT) | Це мінімальна висота над домашньою позицією, яку система завжди буде дотримуватися в режимі утримання, якщо перемикається в цей режим без вказівки висоти (наприклад, за допомогою перемикача на дистанційному керуванні).                                                                            |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE)                      | Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to [Position mode](../flight_modes_mc/position.md). Це можна окремо увімкнути для автоматичних режимів та для режиму поза бортом, і в автоматичних режимах воно включено за замовчуванням. |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV) | Кількість рухів стиків, яка викликає перехід у [режим Положення](../flight_modes_mc/position.md) (якщо [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) увімкнено).                                                                                                      |

<!-- Code for this here: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/loiter.cpp#L61 -->

## Дивіться також

[Hold Mode (FW)](../flight_modes_fw/hold.md)
