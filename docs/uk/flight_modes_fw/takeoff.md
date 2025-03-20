# Режим зльоту (з фіксованим крилом)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Takeoff_ flight mode causes the vehicle to take off to a specified height and then enter [Hold mode](../flight_modes_fw/takeoff.md).

Vehicles are [hand or catapult launched](#catapult-hand-launch) by default, but can also be [configured](#RWTO_TKOFF) to use a [runway takeoff](#runway-takeoff) when supported by the hardware.

::: info

- Режим автоматичний - для керування апаратом не потрібно втручання користувача.
- Режим потребує принаймні дійсної локальної оцінки позиції (не потребує глобальної позиції).
  - Літаючі транспортні засоби не можуть переключатися на цей режим без глобального положення.
  - Літаючі транспортні засоби перейдуть в режим аварійної безпеки, якщо втратять оцінку положення.
  - Роззброєні транспортні засоби можуть переключатися в режим без дійсної оцінки позиції, але не можуть озброюватися.
- Перемикачі радіокерування можна використовувати для зміни режимів польоту.
- Рух стіка радіокерування ігнорується при зліті за допомогою катапульти, але може бути використана для легкого перекочування транспортного засобу при зльоті зі злітної смуги.
- The [Failure Detector](../config/safety.md#failure-detector) will automatically stop the engines if there is a problem on takeoff.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Технічний підсумок

Takeoff mode (and [fixed wing mission takeoff](../flight_modes_fw/mission.md#mission-takeoff)) has two modalities: [catapult/hand-launch](#catapult-hand-launch) or [runway takeoff](#runway-takeoff) (hardware-dependent).
The mode defaults to catapult/hand launch, but can be set to runway takeoff by setting [RWTO_TKOFF](#RWTO_TKOFF) to 1.

To use _Takeoff mode_ you first switch to the mode, and then arm the vehicle.
Прискорення запуску з руки/катапульти спричиняє запуск двигунів.
Для запуску на злітну смугу мотори автоматично посилюються, як тільки транспортний засіб був увімкнений.

Незалежно від модальності, шлях польоту (початкова точка та курс взльоту) та висота дозволу визначені:

- Точкою виходу є позиція транспортного засобу, коли спочатку ввімкнений режим зльоту.
- Курс встановлено на напрям руху транспортного засобу при загорянні
- The clearance altitude is set to [MIS_TAKEOFF_ALT](#MIS_TAKEOFF_ALT).

On takeoff, the aircraft will follow line defined by the starting point and course, climbing at the maximum climb rate ([FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX)) until reaching the clearance altitude.
Reaching the clearance altitude causes the vehicle to enter [Hold mode](../flight_modes_fw/takeoff.md).

### Параметри

Параметри, які впливають як на катапульту/ручний старт, так і на зліт зі злітно-посадкової смуги:

| Параметр                                                                                                                                                                   | Опис                                                                                                                                                                                          |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MIS_TAKEOFF_ALT"></a>[MIS_TAKEOFF_ALT](../advanced_config/parameter_reference.md#MIS_TAKEOFF_ALT)                         | Мінімальна висота встановлення над будинком, на яку підніметься транспортний засіб під час зльоту.                                                                            |
| <a id="FW_TKO_AIRSPD"></a>[FW_TKO_AIRSPD](../advanced_config/parameter_reference.md#FW_TKO_AIRSPD)                               | Takeoff airspeed (is set to [FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN) if not defined by operator) |
| <a id="FW_TKO_PITCH_MIN"></a>[FW_TKO_PITCH_MIN](../advanced_config/parameter_reference.md#FW_TKO_PITCH_MIN) | Це мінімальний кут нахилу заданий під час фази зльоту                                                                                                                                         |
| <a id="FW_T_CLMB_MAX"></a>[FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX)          | Максимальний кут підйому.                                                                                                                                                     |
| <a id="FW_FLAPS_TO_SCL"></a>[FW_FLAPS_TO_SCL](../advanced_config/parameter_reference.md#FW_FLAPS_TO_SCL)    | Налаштування закрилок під час зльоту                                                                                                                                                          |

:::info
The vehicle always respects normal FW max/min throttle settings during takeoff ([FW_THR_MIN](../advanced_config/parameter_reference.md#FW_THR_MIN), [FW_THR_MAX](../advanced_config/parameter_reference.md#FW_THR_MAX)).
:::

<a id="hand_launch"></a>

## Катапульта/ручний запуск

In _catapult/hand-launch mode_ the vehicle waits to detect launch (based on acceleration trigger).
On launch it enables the motor(s) and climbs with the maximum climb rate [FW_T_CLMB_MAX](#FW_T_CLMB_MAX) while keeping the pitch setpoint above [FW_TKO_PITCH_MIN](#FW_TKO_PITCH_MIN).
Once it reaches [MIS_TAKEOFF_ALT](#MIS_TAKEOFF_ALT) it will automatically switch to [Hold mode](../flight_modes_fw/hold.md) and loiter.

Всі рухи стіку радіоуправління ігноруються під час повного взлітного процесу.

Для запуску в цьому режимі:

1. Увімкніть дрон
2. Put the vehicle into _Takeoff mode_
3. Запустіть / киньте транспортний засіб (міцно) безпосередньо у вітер.
  Ви також можете спершу потрясти транспортний засіб, зачекати, поки рушить двигун, а потім кинути його

### Параметри (виявник запуску)

The _launch detector_ is affected by the following parameters:

| Параметр                                                                                                                                                                   | Опис                                                                                                                  |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| <a id="FW_LAUN_DETCN_ON"></a>[FW_LAUN_DETCN_ON](../advanced_config/parameter_reference.md#FW_LAUN_DETCN_ON) | Увімкнути автоматичне визначення запуску. Якщо вимкнені двигуни обертаються при підготовці до польоту |
| <a id="FW_LAUN_AC_THLD"></a>[FW_LAUN_AC_THLD](../advanced_config/parameter_reference.md#FW_LAUN_AC_THLD)    | Поріг прискорення (прискорення в напрямку руху тіла повинно бути вище цієї величини)               |
| <a id="FW_LAUN_AC_T"></a>[FW_LAUN_AC_T](../advanced_config/parameter_reference.md#FW_LAUN_AC_T)             | Час спрацьовування (прискорення повинно бути вище порогу на цю кількість секунд)                   |
| <a id="FW_LAUN_MOT_DEL"></a>[FW_LAUN_MOT_DEL](../advanced_config/parameter_reference.md#FW_LAUN_MOT_DEL)    | Затримка від виявлення запуску до відкручування мотору                                                                |

<a id="runway_launch"></a>

## Взліт зі злітної смуги

Зліт зі злітної смуги можна використовувати тільки для транспортних засобів з посадковим шасі та керованим колесом.
You will first need to enable the wheel controller using the parameter [FW_W_EN](#FW_W_EN).

Транспортний засіб повинен бути в центрі та вирівняний по злітній смузі, коли починається зльот.
The operator can "nudge" the vehicle while on the runway to help keeping it centered and aligned (see [RWTO_NUDGE](../advanced_config/parameter_reference.md#RWTO_NUDGE)).

The _runway takeoff mode_ has the following phases:

1. **Throttle ramp**: Throttle is ramped up within [RWTO_RAMP_TIME](../advanced_config/parameter_reference.md#RWTO_RAMP_TIME) to [RWTO_MAX_THR](../advanced_config/parameter_reference.md#RWTO_MAX_THR).
2. **Clamped to runway**: Pitch fixed, no roll and takeoff path controlled until the rotation airspeed ([RWTO_ROT_AIRSPD](../advanced_config/parameter_reference.md#RWTO_ROT_AIRSPD)) is reached. Оператор може підганяти транспортний засіб ліворуч/праворуч за допомогою стіка рискання.
3. **Climbout**: Increase pitch setpoint and climb to takeoff altitude. Щоб уникнути ударів крил, контролер буде тримати встановлений кут кочення заблокованим на 0, коли близько до землі, а потім поступово дозволить більше кочення під час підйому. It is based on the vehicle geometry as configured in [FW_WING_SPAN](#FW_WING_SPAN) and [FW_WING_HEIGHT](#FW_WING_HEIGHT).

:::info
For a smooth takeoff, the runway wheel controller possibly needs to be tuned.
It consists of a rate controller (P-I-FF-controller with the parameters [FW_WR_P](../advanced_config/parameter_reference.md#FW_WR_P), [FW_WR_I](../advanced_config/parameter_reference.md#FW_WR_I), [FW_WR_FF](../advanced_config/parameter_reference.md#FW_WR_FF)) and an outer loop that calculates heading setpoints from course errors and can be tuned via [RWTO_NPFG_PERIOD](#RWTO_NPFG_PERIOD).
:::

### Параметри (зліт зі злітної смуги)

Зліт зі злітної смуги залежить від наступних параметрів:

| Параметр                                                                                                                                              | Опис                                                                                                                                                                                                                                |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RWTO_TKOFF"></a>[RWTO_TKOFF](../advanced_config/parameter_reference.md#RWTO_TKOFF)                                        | Увімкніть зліт по взлітній смузі                                                                                                                                                                                                    |
| <a id="FW_W_EN"></a>[FW_W_EN](../advanced_config/parameter_reference.md#FW_W_EN)                            | Увімкнути контролер колеса                                                                                                                                                                                                          |
| <a id="RWTO_MAX_THR"></a>[RWTO_MAX_THR](../advanced_config/parameter_reference.md#RWTO_MAX_THR)             | Максимальне розгін під час взліту зі злітної смуги                                                                                                                                                                                  |
| <a id="RWTO_RAMP_TIME"></a>[RWTO_RAMP_TIME](../advanced_config/parameter_reference.md#RWTO_RAMP_TIME)       | Час прискорення ручки газу                                                                                                                                                                                                          |
| <a id="RWTO_ROT_AIRSPD"></a>[RWTO_ROT_AIRSPD](../advanced_config/parameter_reference.md#RWTO_ROT_AIRSPD)    | Поріг швидкості для початку підняття (нахилення вгору). Якщо не налаштовано оператором, встановлюється на 0,9\*FW_TKO_AIRSPD.          |
| <a id="RWTO_ROT_TIME"></a>[RWTO_ROT_TIME](../advanced_config/parameter_reference.md#RWTO_ROT_TIME)          | Це час, який необхідно лінійно нарощувати обмеження швидкості прийому під час обертання на зльоті.                                                                                                                  |
| <a id="FW_TKO_AIRSPD"></a>[FW_TKO_AIRSPD](../advanced_config/parameter_reference.md#FW_TKO_AIRSPD)          | Задана швидкість під час розгону під час зльоту (після відкочування). Якщо не налаштовано оператором, встановлюється на FW_AIRSPD_MIN. |
| <a id="RWTO_NUDGE"></a>[RWTO_NUDGE](../advanced_config/parameter_reference.md#RWTO_NUDGE)                                        | Увімкніть керування колесом під час руху по злітній смузі                                                                                                                                                                           |
| <a id="FW_WING_SPAN"></a>[FW_WING_SPAN](../advanced_config/parameter_reference.md#FW_WING_SPAN)             | Розмах крила транспортного засобу. Використовується для запобігання ударів крилом.                                                                                                                  |
| <a id="FW_WING_HEIGHT"></a>[FW_WING_HEIGHT](../advanced_config/parameter_reference.md#FW_WING_HEIGHT)       | Висота крил над землею (дорожній просвіт). Використовується для запобігання ударів крилом.                                                                                       |
| <a id="RWTO_NPFG_PERIOD"></a>[RWTO_NPFG_PERIOD](../advanced_config/parameter_reference.md#RWTO_NPFG_PERIOD) | Час L1 під час керування на злітній смузі. Збільшення для менш агресивної відповіді на помилки курсу.                                                                                               |

## Дивіться також

- [Takeoff Mode (MC)](../flight_modes_mc/takeoff.md)
- [Planning a mission takeoff](../flight_modes_fw/mission.md#mission-takeoff)

<!-- this maps to AUTO_TAKEOFF in dev -->
