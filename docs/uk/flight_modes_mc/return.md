# Режим повернення (мультикоптер)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

Режим польоту _Return_ використовується для _повернення транспортного засобу до безпеки_ по вільному шляху до безпечного пункту призначення, де він може приземлитися.

Мультикоптери за замовчуванням використовують тип повертання до [домашньої/ралійної точки](../flight_modes/return.md#home-rally-point-return-type-rtl-type-0).
У цьому типі повернення транспортні засоби підіймаються на безпечну висоту над перешкодами, якщо потрібно, прямують безпосередньо до найближчої безпечної точки посадки (точки збору або домашньої позиції), опускаються на "висоту опускання", надвіюються коротко, а потім сідають.
Висота повернення, висота зниження та затримка при посадці зазвичай встановлені на консервативні "безпечні" значення, але їх можна змінити за потреби.

Multicopter підтримує [інші типи повернення PX4](../flight_modes/return.md#return-types-rtl-type), включаючи посадку місії, маршрут місії і найближче безпечне місце призначення.
За замовчуванням рекомендується використовувати цей тип.

::: info

- Режим автоматичний - для керування апаратом не потрібно втручання користувача.
- Режим вимагає глобальної оцінки 3D-позиції (з GPS або виведеної з [локальної позиції](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Літаючі транспортні засоби не можуть переключатися на цей режим без глобального положення.
  - Літаючі транспортні засоби перейдуть в режим аварійної безпеки, якщо втратять оцінку положення.
- Режим вимагає встановленої домашньої позиції.
- Mode prevents arming (vehicle cannot be armed while this mode is selected).
- Перемикачі керування RC можуть використовуватися для зміни режимів польоту на будь-якому транспортному засобі.
- RC stick movement will [by default](#COM_RC_OVERRIDE) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless prevented by the active failsafe state.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Технічний підсумок

Multicopters use the [home/rally point return type](../flight_modes/return.md#home-rally-point-return-type-rtl-type-0) by default ([RTL_TYPE=0](../advanced_config/parameter_reference.md#RTL_TYPE)).
У цьому типі повернення вертольот:

- Піднімається на [мінімальну висоту повернення](#minimum-return-altitude) (безпечно вище будь-яких очікуваних перешкод).
  Транспортний засіб підтримує свою початкову висоту, якщо вона вище, ніж мінімальна висота повернення.
- Летить за допомогою прямого постійного шляху на безпечну точку посадки, яка буде найближчою із всіх ралі-точок та домашньої позиції.
- Прибуваючи до пункту призначення, він швидко спускається на «висоту спуску» ([RTL_DESCEND_ALT](#RTL_DESCEND_ALT)).
- Він чекає протягом налаштованого часу ([RTL_LAND_DELAY](#RTL_LAND_DELAY)), який може бути використаний для розгортання шасі посадки.
- Потім сідає на землю.

### Мінімальна висота повернення

За замовчуванням _мінімальна висота повернення_ встановлюється за допомогою [RTL_RETURN_ALT](#RTL_RETURN_ALT), і транспортний засіб просто повернеться на вищу з висоти `RTL_RETURN_ALT` або початкової висоти транспортного засобу.

The minimum return altitude can be further configured using [RTL_CONE_ANG](#RTL_CONE_ANG) and [RTL_MIN_DIST](#RTL_MIN_DIST), which together with [RTL_RETURN_ALT](#RTL_RETURN_ALT) define a half cone centered around the destination landing point.
Within `RTL_MIN_DIST` of the destination, the return altitude is calculated from the cone geometry rather than set directly to `RTL_RETURN_ALT`, allowing a lower minimum return altitude when close to the destination.
This is useful when there are few obstacles near the destination, as it may reduce the height the vehicle needs to ascend before landing, and hence power consumption and time to land.

![Режим повернення конуса](../../assets/flying/rtl_cone.jpg)

The cone affects the minimum return altitude if return mode is triggered within the cylinder defined by the maximum cone radius ([RTL_MIN_DIST](#RTL_MIN_DIST)) and `RTL_RETURN_ALT`: outside this cylinder `RTL_RETURN_ALT` is used.
Inside the cone, the vehicle returns at an altitude calculated from the cone geometry, up to `RTL_RETURN_ALT`.
After reaching the destination, it descends to `RTL_DESCEND_ALT` (if above that altitude) before landing or loitering.

Для отримання додаткової інформації про цей тип повернення див. [Тип повернення додому/району збору (RTL_TYPE=0)](../flight_modes/return.md#home-rally-point-return-type-rtl-type-0)

## Параметри

Параметри RTL перелічені в [Референсі параметрів > Режим повернення](../advanced_config/parameter_reference.md#return-mode).

Параметри, які є важливими для мультикоптерів (припускаючи, що [RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) встановлено на 0), перераховані нижче.

| Parameter                                                                                                                                                               | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RTL_RETURN_ALT"></a>[RTL_RETURN_ALT](../advanced_config/parameter_reference.md#RTL_RETURN_ALT)                         | Повернути висоту в метрах (за замовчуванням: 60м), коли [RTL_CONE_ANG](../advanced_config/parameter_reference.md#RTL_CONE_ANG) дорівнює 0. Якщо вже вище цієї величини, транспортний засіб повернеться на поточну висоту.                                                                                                                                                                                                                                                |
| <a id="RTL_DESCEND_ALT"></a>[RTL_DESCEND_ALT](../advanced_config/parameter_reference.md#RTL_DESCEND_ALT)                      | Altitude above the destination used for the final descent before landing or loitering (default: 30m).                                                                                                                                                                                                                                                                                                                                                                                                                              |
| <a id="RTL_LAND_DELAY"></a>[RTL_LAND_DELAY](../advanced_config/parameter_reference.md#RTL_LAND_DELAY)                         | Time to hover at `RTL_DESCEND_ALT` before landing (default: 0.5s) - by default this period is short so that the vehicle will simply slow and then land immediately. Якщо встановлено значення -1, система буде кружляти на висоті `RTL_DESCEND_ALT` замість посадки. Затримка надається для того, щоб ви могли налаштувати час для розгортання шасі для посадки (автоматично спрацьовує).                                                                       |
| <a id="RTL_MIN_DIST"></a>[RTL_MIN_DIST](../advanced_config/parameter_reference.md#RTL_MIN_DIST)                               | Within this distance from the home position, the return altitude is calculated from the "cone" rather than directly from `RTL_RETURN_ALT`.                                                                                                                                                                                                                                                                                                                                                                                                                            |
| <a id="RTL_CONE_ANG"></a>[RTL_CONE_ANG](../advanced_config/parameter_reference.md#RTL_CONE_ANG)                               | Половина кута конуса, який визначає висоту повернення транспортного засобу RTL. Values (in degrees): `0`, `25`, `45`, `65`, `80`, `90`. Note that `0` is "no cone" (always return at `RTL_RETURN_ALT` or higher), while `90` indicates an almost vertical cone, so the vehicle generally returns at its current altitude when close to the destination. The return altitude may still be constrained to avoid flying too low while approaching the destination. |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE)                      | Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to [Position mode](../flight_modes_mc/position.md) (unless prevented by the active failsafe state). Це можна окремо увімкнути для автоматичних режимів та для режиму поза бортом, і в автоматичних режимах воно включено за замовчуванням.                                                                                                                                                                           |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV) | The amount of stick movement that causes a transition to [Position mode](../flight_modes_mc/position.md) (if [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) is enabled).                                                                                                                                                                                                                                                                                                                                            |

## Дивіться також

- [Режим повернення (Загальний)](../flight_modes/return.md)
- [Режим повернення (з нерухомим крилом)](../flight_modes_fw/return.md)
- [Режим повернення (VTOL)](../flight_modes_vtol/return.md)
