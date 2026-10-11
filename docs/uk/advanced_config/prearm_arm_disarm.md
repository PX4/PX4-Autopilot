# Arm, Disarm, Prearm Конфігурація

Транспортні засоби можуть мати рухомі частини, деякі з яких можуть бути потенційно небезпечними під час роботи (особливо мотори та пропелери)!

Для зменшення ймовірності аварій, PX4 має явні стани для включення компонентів транспортного засобу:

- **Вимкнено:** Немає живлення для моторів або приводів.
- **Передпускний стан:** Мотори/пропелери заблоковані, але приводи для не небезпечної електроніки живлені (наприклад, елерони, закрилки і т. д.).
- **Озброєно:** Транспортний засіб повністю увімкнено. Двигуни/пропелери можуть обертатися (небезпечно!)

:::info
Наземні станції можуть відображати _вимкнено_ для транспортних засобів у режимі передпуску.
Хоча це не є технічно правильним для транспортних засобів у режимі передпуску, це "безпечно".
:::

Користувачі можуть керувати переходом між цими станами, використовуючи [захисний перемикач](../getting_started/px4_basic_concepts.md#safety-switch) на транспортному засобі (за бажанням) _та_ перемикач/кнопку [озброєння](#arm_disarm_switch), [жест озброєння](#arm_disarm_gestures) або _команду MAVLink_ на наземному контролері:

- A _safety switch_ is a control _on the vehicle_ that must be engaged before the vehicle can be armed, and which may also prevent pre-arming (depending on the configuration).
  Зазвичай захисний перемикач інтегровано у блок GPS, але він також може бути окремим фізичним компонентом.

  Транспортний засіб, який озброєний, потенційно небезпечний.
  Захисний перемикач - це додатковий механізм, який запобігає випадковому озброєнню.

:::

- Перемикач _озброєння_ - це перемикач або кнопка _на пульті керування RC_, який може бути використаний для озброєння транспортного засобу та запуску моторів (якщо озброєння не заборонене захисним перемикачем).

- Жест озброєння - це рух педалей _на пульті керування RC_, який може бути використаний як альтернатива перемикачу озброєння.

- MAVLink commands can also be sent by a ground control station to pre-arm, arm, or disarm a vehicle.

PX4 також автоматично вимикає транспортний засіб, якщо він не злітає протягом певного часу після озброєння, і якщо він не вимикається вручну після посадки.
Це зменшує час, коли на землі знаходиться озброєний (і, отже, небезпечний) транспортний засіб.

PX4 дозволяє налаштовувати роботу передпуску, озброєння та вимикання за допомогою параметрів (які можна редагувати в _QGroundControl_ за допомогою [редактора параметрів](../advanced_config/parameters.md)), як описано у наступних розділах.

:::tip
Параметри озброєння/вимикання можна знайти у [Посилання на параметри > Командир](../advanced_config/parameter_reference.md#commander) (шукайте `COM_ARM_*` та `COM_DISARM_*`).
:::

## Arming/Disarming Gestures {#arm_disarm_gestures}

За замовчуванням, транспортний засіб озброюється та вимикається шляхом виконання певних рухів педалями газу/рулем в маневрів та утримання їх протягом 1 секунди.

- **Озброєння:** Мінімальна педаль газу, максимальний рульовий вектор
- **Вимикання:** Мінімальна педаль газу, мінімальний рульовий вектор

RC controllers will use different sticks for throttle and yaw [based on their mode](../getting_started/rc_transmitter_receiver.md#types-of-remote-controllers), and hence different gestures:

- **Режим 2**:
  - _Озброєння:_ Ліва педаль вниз і вправо.
  - _Вимкнення:_ Ліва педаль вниз і вліво.
- **Режим 1**:
  - _Озброєння:_ Ліва педаль вправо, права педаль вниз.
  - _Вимкнення:_ Ліва педаль вліво, права педаль вниз.

Note that disarming in any altitude controlled mode is only possible after landing was detected.
In manually piloted modes without altitude control, such as Stabilized, Acro, and Manual, it's always possible to disarm using gestures or buttons — even in flight.

| Parameter                                                                                                                                          | Опис                                                                                                                                                        |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MAN_ARM_GESTURE"></a>[MAN_ARM_GESTURE](../advanced_config/parameter_reference.md#MAN_ARM_GESTURE) | Enable arm/disarm stick guesture. `0`: Disabled, `1`: Enabled (default). |

## Arming Button/Switch {#arm_disarm_switch}

Кнопку _озброєння_ або "моментальний перемикач" можна налаштувати для спрацьовування озброєння/вимикання _замість_ [озброєння за допомогою жестів](#arm_disarm_gestures) (встановлення перемикача озброєння вимикає озброєння за допомогою жестів).
The button should be held down for one second to arm (when disarmed) or disarm (when armed).

Двопозиційний перемикач також може використовуватися для озброєння/вимикання, при цьому відповідні команди на озброєння/вимкнення надсилаються при _перемиканні_ перемикача.

:::tip
Двопозиційні перемикачі для озброєння переважно використовуються в/рекомендовані для гоночних дронів.
:::

Перемикач або кнопка призначається (та активується) за допомогою [RC_MAP_ARM_SW](#RC_MAP_ARM_SW), а тип перемикача налаштовується за допомогою [COM_ARM_SWISBTN](#COM_ARM_SWISBTN).

| Parameter                                                                                                                                                         | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RC_MAP_ARM_SW"></a>[RC_MAP_ARM_SW](../advanced_config/parameter_reference.md#RC_MAP_ARM_SW) | Канал перемикача озброєння радіокерування (типове значення: 0 - не призначено). Якщо визначено, вказаний канал радіокерування (кнопка/перемикач) використовується для озброєння замість жесту палиці. <br>**Note:**<br>- This setting _disables the stick gesture_!<br>- This setting applies to RC controllers. It does not apply to Joystick controllers that are connected via _QGroundControl_. |
| <a id="COM_ARM_SWISBTN"></a>[COM_ARM_SWISBTN](../advanced_config/parameter_reference.md#COM_ARM_SWISBTN)                | Перемикач озброєння є моментальною кнопкою. <br>- `0`: Arm switch is a 2-position switch where arm/disarm commands are sent on switch transitions.<br>-`1`: Arm switch is a momentary button where the arm/disarm command is sent after holding down the button for one second.                                                                                                                                                                           |

:::info
Перемикач також можна налаштувати як частину конфігурації _QGroundControl_ для [Режиму польоту](../config/flight_mode.md).
:::

## Автоматичне вимкнення

За замовчуванням транспортні засоби автоматично вимикаються при посадці або якщо ви заберете занадто багато часу, щоб злітати після озброєння.
Ця функція налаштовується за допомогою наступних таймаутів.

| Parameter                                                                                                                                             | Опис                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_DISARM_LAND"></a>[COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND)    | Час очікування для автоматичного відбрасування після приземлення. За замовчуванням: 2с (значення -1, щоб вимкнути). |
| <a id="COM_DISARM_PRFLT"></a>[COM_DISARM_PRFLT](../advanced_config/parameter_reference.md#COM_DISARM_PRFLT) | Час очікування для автоматичного відбрасування, якщо занадто повільно підйом. Default: 10s (-1 to disable).         |

By default, the vehicle keeps safety off after disarming.
If [COM_FORCE_SAFETY](#COM_FORCE_SAFETY) is set to `1`, safety is re-enabled on disarm, so it must be turned off again (by switch or MAVLink command, depending on [COM_SAFETY_MODE](#COM_SAFETY_MODE)) before the next arming.
In modes that pre-arm when safety is turned off, this also exits the pre-armed state.
This parameter has no effect when `COM_SAFETY_MODE` is set to `0`.

| Parameter                                                                                                                                             | Опис                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_FORCE_SAFETY"></a>[COM_FORCE_SAFETY](../advanced_config/parameter_reference.md#COM_FORCE_SAFETY) | Re-enable safety when the vehicle disarms. Default: `0` (Disabled). |

## Auto-Arming on Boot

The vehicle can be configured to arm automatically on boot once all preflight checks pass,
using the `COM_ARM_ON_BOOT` parameter. For safety, PX4 enforces a minimum 5-second delay after boot before attempting to arm.

Once armed this way, the vehicle will not re-arm automatically after a manual disarm.

:::info
The parameter value is read once at boot.
Changing it while the system is running has no effect until the next reboot.
:::

:::warning
Use with caution.
A vehicle that arms automatically can spin up motors and actuators without any operator gesture.
Ensure the vehicle is in a safe state before powering on.
:::

| Parameter                                                                                                                                                               | Опис                                                                                                                                                 |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_ARM_ON_BOOT"></a>[COM_ARM_ON_BOOT](../advanced_config/parameter_reference.md#COM_ARM_ON_BOOT) | Arm automatically once preflight checks pass after boot. Default: `0` (Disabled). |

## Pre-Arm Checks {#prearm_checks}

To reduce accidents, vehicles are only allowed to arm certain conditions are met (some of which are configurable).
Армування заборонено у таких випадках:

- Повітряне судно не перебуває у "здоровому" стані.
  Наприклад, воно не калібрується або має помилки датчиків.
- The vehicle still has its [safety state](#safety_state) set to _ON_. This could for instance be a [safety switch](../getting_started/px4_basic_concepts.md#safety-switch) that has not been engaged.
- The vehicle has a [remote ID](../peripherals/remote_id.md) that is unhealthy or otherwise not ready
- VTOL-повітряне судно перебуває в режимі фіксованого крила ([by default(за замовчуванням)](../advanced_config/parameter_reference.md#CBRK_VTOLARMING)).
- Поточний режим потребує належної глобальної позиційної оцінки, але повітряне судно не має блокування GPS.
- Many more (see [arming/disarming safety settings](../config/safety.md#arming-disarming-settings) for more information).

The current failed checks can be viewed in QGroundControl (v4.2.0 and later) [Arming Check Report](../flying/pre_flight_checks.md#qgc-arming-check-report) (see also [Fly View > Toolbar > Flight Status](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view_toolbar.html#flight-status)).

Зауважте, що внутрішньо PX4 перевіряє активацію на 10 Гц.
Список невдалих перевірок зберігається, і якщо цей список змінюється, PX4 видає поточний список за допомогою [інтерфейсу подій](../concept/events_interface.md).
Список також надсилається, коли GCS підключається.
Effectively the GCS knows the status of pre-arm checks immediately, both when disarmed and armed.

:::details
Implementation notes for developers
Примітки для розробників Реалізація клієнта знаходиться у [libevents](https://github.com/mavlink/libevents):

- [libevents > Групи подій](https://github.com/mavlink/libevents#event-groups)
- [health_and_arming_checks.h](https://github.com/mavlink/libevents/blob/main/libs/cpp/parse/health_and_arming_checks.h)

QGC реалізація: [HealthAndArmingCheckReport.cc](https://github.com/mavlink/qgroundcontrol/blob/master/src/MAVLink/LibEvents/HealthAndArmingCheckReport.cc).
:::

PX4 також видає підмножину інформації перевірки постановки на охорону в повідомленні [SYS_STATUS](https://mavlink.io/en/messages/common.html#SYS_STATUS) (див. [MAV_SYS_STATUS_SENSOR](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR)).

## Arming Sequence: Safety State {#safety_state}

The arming sequence depends on whether or not there is a _safety switch_, and is controlled by the parameter [COM_SAFETY_MODE](#COM_SAFETY_MODE). Changes to the parameter only take effect after a reboot.

When disarmed (or pre-armed), the safety state can either be _ON_ (a.k.a. _SAFE_), or _OFF_ (a.k.a. _DANGEROUS_). When it is _ON_, arming will always be prevented. When it is _OFF_, the vehicle can be armed for as long as the [pre-arm checks](#prearm_checks) have passed.

Additionally, the [COM_PREARM_MODE](#COM_PREARM_MODE) parameter defines when/if pre-arm mode is enabled ("safe"/non-throttling actuators are able to move):

- `Disabled` (Default): Pre-arm mode disabled (there is no stage where only non-throttling actuators are enabled).
- `When safety off`: Pre-arm mode is enabled when safety is turned off.
- `Always`: Pre-arm mode is enabled from power up.

The sections below detail the startup sequences for the different configurations of [COM_SAFETY_MODE](#COM_SAFETY_MODE) and [COM_PREARM_MODE](#COM_PREARM_MODE).

### COM_SAFETY_MODE=Always off (Default) and COM_PREARM_MODE=Disabled (Default)

The default configuration does not impose any additional safety measures. Arming is possible as soon as the rest of the system is ready.

Послідовність запуску така:

1. Увімкнення живлення.
   - Усі приводи заблоковано у беззбройному(вимкненому) положенні
   - System safety is off: Arming possible once the other pre-arm checks pass.
2. Видається команда на озброєння(збурення).
   - Система озброєна(збурена).
   - Усі мотори та приводи можуть рухатися.

### COM_SAFETY_MODE=Safety switch (physical or virtual via MAVLink) and COM_PREARM_MODE=When safety off

This configuration lets you use either the safety switch or a MAVLink command [MAV_CMD_DO_SET_SAFETY_SWITCH_STATE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SAFETY_SWITCH_STATE) to turn safety off. Note that sending the corresponding MAVLink command with SAFET&#x59;_&#x53;WITCH_STATE_SAFE also lets you turn safety back on, but the physical switch does \_not_ allow to go back to a safe state.

Послідовність запуску така:

1. Увімкнення живлення.
   - Усі приводи заблоковано у беззбройному(вимкненому) положенні
   - Неможливо озброїти(збурити).
2. Safety switch is pressed or a MAVLink command is received.
   - System now pre-armed: non-throttling actuators can move (e.g. ailerons).
   - System safety is off: Arming possible once the other pre-arm checks pass.
3. Видається команда на озброєння(збурення).
   - Система озброєна(збурена).
   - Усі мотори та приводи можуть рухатися.

### COM_SAFETY_MODE=Physical safety switch only and COM_PREARM_MODE=When safety off

When safety mode is `Physical safety switch only`, you must press the safety switch to turn safety off. Note that pressing the safety switch again does _not_ allow to go back to a safe state.
The MAVLink command is rejected.

Послідовність запуску така:

1. Увімкнення живлення.
   - Усі приводи заблоковано у беззбройному(вимкненому) положенні
   - Неможливо озброїти(збурити).
2. Перемикання безпеки натиснуто.
   - System now pre-armed: non-throttling actuators can move (e.g. ailerons).
   - System safety is off: Arming possible once the other pre-arm checks pass.
3. Видається команда на озброєння(збурення).
   - Система озброєна(збурена).
   - Усі мотори та приводи можуть рухатися.

### COM_SAFETY_MODE=MAVLink only and COM_PREARM_MODE=When safety off

When safety mode is `MAVLink only`, you must send a MAVLink command [MAV_CMD_DO_SET_SAFETY_SWITCH_STATE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SAFETY_SWITCH_STATE) to turn safety off. Note that sending the corresponding MAVLink command with SAFETY_SWITCH_STATE_SAFE also lets you turn safety back on.
Any physical safety switch is ignored.

Послідовність запуску така:

1. Увімкнення живлення.
   - Усі приводи заблоковано у беззбройному(вимкненому) положенні
   - Неможливо озброїти(збурити).
2. A MAVLink command is received.
   - System now pre-armed: non-throttling actuators can move (e.g. ailerons).
   - System safety is off: Arming possible once the other pre-arm checks pass.
3. Видається команда на озброєння(збурення).
   - Система озброєна(збурена).
   - Усі мотори та приводи можуть рухатися.

### Параметри

| Parameter                                                                                                                                          | Опис                                                                                                                   |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_SAFETY_MODE"></a>[COM_SAFETY_MODE](../advanced_config/parameter_reference.md#COM_SAFETY_MODE) | Condition to turn safety off. Vehicle arming is prevented for as long as safety is on. |
| <a id="COM_PREARM_MODE"></a>[COM_PREARM_MODE](../advanced_config/parameter_reference.md#COM_PREARM_MODE) | Condition to enter the prearmed state.                                                                 |

<!-- Discussion:
https://github.com/PX4/PX4-Autopilot/pull/12806#discussion_r318337567
https://github.com/PX4/PX4-user_guide/issues/567#issue-486653048
-->
