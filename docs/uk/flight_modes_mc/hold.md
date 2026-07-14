# Режим Зависання або Hold (Мультикоптер)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Hold_ flight mode causes the vehicle to stop and hover, maintaining its position and altitude.

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
- Stick movement will [by default](#MAN_OVERRIDE_SPD) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless prevented by the active failsafe state.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Технічний підсумок

The vehicle stops and hovers, maintaining its position and altitude.
The vehicle will first ascend to [NAV_MIN_LTR_ALT](#NAV_MIN_LTR_ALT) if the mode is engaged below this altitude.

Stick movement will change the vehicle to [Position mode](../flight_modes_mc/position.md) (by [default](#MAN_OVERRIDE_SPD)).

### Параметри

Поведінку режиму утримання можна налаштувати за допомогою наведених нижче параметрів.

| Parameter                                                                                                                                                               | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="NAV_MIN_LTR_ALT"></a>[NAV_MIN_LTR_ALT](../advanced_config/parameter_reference.md#NAV_MIN_LTR_ALT) | Це мінімальна висота над домашньою позицією, яку система завжди буде дотримуватися в режимі утримання, якщо перемикається в цей режим без вказівки висоти (наприклад, за допомогою перемикача на дистанційному керуванні).                                                                                                                                                                                                                                                                                                                                                                                          |
| <a id="MAN_OVERRIDE_SPD"></a>[MAN_OVERRIDE_SPD](../advanced_config/parameter_reference.md#MAN_OVERRIDE_SPD)                   | Speed (normalized stick travel per second) above which moving the sticks controlling a multicopter (or VTOL in hover) gives control back to the pilot by switching to [Position mode](../flight_modes_mc/position.md) (or Altitude mode if position is unavailable). At the default value of 1 a half-stick movement in ~0.5 s triggers it; lower is more sensitive. A stick held statically has zero speed and will not trigger. Set to -1 to disable. <Badge type="tip" text="PX4 v1.18" /> |
| <a id="MPC_AUTO_NUDGING"></a>[MPC_AUTO_NUDGING](../advanced_config/parameter_reference.md#MPC_AUTO_NUDGING)                   | Bitmask enabling stick nudging in Auto modes (multicopter). Bit 0 (yaw nudging) lets the yaw stick nudge heading in Hold, held when the stick is released; switching flight mode clears the held heading. Requires stick override to be disabled ([MAN_OVERRIDE_SPD](#MAN_OVERRIDE_SPD) = -1).                                                                                                                                                                                                      |

<!-- Code for this here: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/loiter.cpp#L61 -->

## Дивіться також

[Hold Mode (FW)](../flight_modes_fw/hold.md)
