# Режим зльоту (Мультикоптер)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Takeoff_ flight mode causes the vehicle to take off to a specified height and wait for further input.

::: info

- Режим автоматичний - для керування апаратом не потрібно втручання користувача.
- Режим потребує принаймні дійсної локальної оцінки позиції (не потребує глобальної позиції).
  - Літаючі транспортні засоби не можуть переключатися на цей режим без глобального положення.
  - Літаючі транспортні засоби перейдуть в режим аварійної безпеки, якщо втратять оцінку положення.
  - Роззброєні транспортні засоби можуть переключатися в режим без дійсної оцінки позиції, але не можуть озброюватися.
- Перемикачі радіокерування можна використовувати для зміни режимів польоту.
- Stick movement will [by default](#MAN_OVERRIDE_SPD) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless prevented by the active failsafe state.
- The [Failure Detector](../config/safety.md#failure-detector) will automatically stop the engines if there is a problem on takeoff.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Технічний підсумок

A multi rotor ascends vertically to the altitude defined in [MIS_TAKEOFF_ALT](../advanced_config/parameter_reference.md#MIS_TAKEOFF_ALT) and holds position.

Stick movement will change the vehicle to [Position mode](../flight_modes_mc/position.md) (by [default](#MAN_OVERRIDE_SPD)).

### Параметри

Взліт впливається наступними параметрами:

| Parameter                                                                                                                                             | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="MIS_TAKEOFF_ALT"></a>[MIS_TAKEOFF_ALT](../advanced_config/parameter_reference.md#MIS_TAKEOFF_ALT)    | Цільова висота під час злітання (типово: 2.5м)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| <a id="MPC_TKO_SPEED"></a>[MPC_TKO_SPEED](../advanced_config/parameter_reference.md#MPC_TKO_SPEED)          | Швидкість підйому (за замовчуванням: 1.5м/с)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| <a id="MAN_OVERRIDE_SPD"></a>[MAN_OVERRIDE_SPD](../advanced_config/parameter_reference.md#MAN_OVERRIDE_SPD) | Speed (normalized stick travel per second) above which moving the sticks controlling a multicopter (or VTOL in hover) gives control back to the pilot by switching to [Position mode](../flight_modes_mc/position.md) (or Altitude mode if position is unavailable). At the default value of 1 a half-stick movement in ~0.5 s triggers it; lower is more sensitive. A stick held statically has zero speed and will not trigger. Set to -1 to disable. <Badge type="tip" text="PX4 v1.18" /> |

## Takeoff From a Moving Platform (Boat Deck) <Badge type="tip" text="PX4 v1.18" />

Automatic takeoff from a moving platform, such as a boat deck, is supported. The multicopter climbs and holds the point where it left the deck, rather than following the boat.

:::warning
Given that the vehicle holds the point where it left the deck and does not travel along with the boat, always take off from the **back (stern) of the boat**, with clear open space behind it.
Taking off from the bow or the middle of the boat can leave it behind across the deck and into masts, antennas, superstructure, or crew.
:::

:::info
Takeoff from a moving platform is only supported in [Takeoff mode](#technical-summary) (or the equivalent coordinate-less takeoff commanded from a ground station), not as part of a [Mission](../flight_modes_mc/mission.md).
To run a mission from a boat, take off first in Takeoff mode and then start the mission once the vehicle is airborne.
:::

The moving-platform case can be exercised in simulation with the Gazebo [Moving Platform world](../sim_gazebo_gz/worlds.md#moving-platform).

## Дивіться також

- [Throw Launch (MC)](../flight_modes_mc/throw_launch.md)
- [Takeoff Mode (FW)](../flight_modes_fw/takeoff.md)

<!-- this maps to AUTO_TAKEOFF in dev -->
