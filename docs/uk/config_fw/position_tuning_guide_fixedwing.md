# Налаштування контролера висоти/положення фіксованого крила

Ця інструкція надає певну допомогу в налаштуванні контролерів високого рівня, необхідних для літаючих місій, та в режимі контролю за висотою.
PX4 використовує TECS для контролю висоти та швидкості повітря, а NPFG - для горизонтальних заголовків/позицій.

:::info
An incorrectly set gain during tuning can make altitude or heading control unstable.
Пілот, налаштовуючи коефіцієнти TECS, повинен мати змогу керувати польотом та посадкою літака в стабілізованому режимі керування.
:::

:::tip
All parameters are documented in the [Parameter Reference](../advanced_config/parameter_reference.md#fw-tecs).
Найважливіші параметри охоплені в цьому керівництві.
:::

## Налаштування TECS (висота і швидкість)

TECS (Total Energy Control System) - це алгоритм керування для літаків з фіксованими крилами, який координує установки керування керуванням та кутом нахилу, щоб контролювати висоту та швидкість літака.
For a detailed description of the TECS algorithm and the control diagram, see [Controller Diagrams](../flight_stack/controller_diagrams.md).

A well-tuned attitude controller is required before tuning TECS: [PID Tuning Guide](../config_fw/pid_tuning_guide_fixedwing.md).

Налаштування TECS полягає головним чином у правильному встановленні обмежень повітряного каркасу.
Ці обмеження можуть бути визначені у термінах параметрів, які можуть бути визначені з послідовності маневрів польоту, які описані нижче.
Most of the maneuvers required the plane to be flown by a pilot in [Stabilized flight mode](../flight_modes_fw/stabilized.md).

:::tip
It is highly beneficial to have a person available who can read and take note of telemetry data while the pilot is flying the maneuvers.
Для підвищення точності ми також рекомендуємо перевірити дані, отримані під час польоту, з даними, записаними в журналах польотів.
:::

#### 1st: Trim Conditions

Fly in [stabilized mode](../flight_modes_fw/stabilized.md) and find trim values for both throttle and pitch angle for level flight at trim airspeed.
Використовуйте дросель для налаштування швидкості повітря та кута для підтримання рівноваги польоту.

Встановить наступні параметри:

- [FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) - set to the desired trim airspeed flown during the maneuver.
- [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM) - set to the throttle required to fly at trim airspeed.
- [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF) - set to the pitch angle required to maintain level flight.

#### 2nd: Airspeed & Throttle Limits

Fly in [stabilized mode](../flight_modes_fw/stabilized.md) and increase throttle while maintaining level flight using pitch control - until the vehicle reaches
the maximum allowed airspeed.

Встановить наступні параметри:

- [FW_THR_MAX](../advanced_config/parameter_reference.md#FW_THR_MAX) - set to the throttle you applied to reach maximum airspeed during level flight.
- [FW_THR_MIN](../advanced_config/parameter_reference.md#FW_THR_MIN) - set to the minimum throttle the plane should fly at.
- [FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX) - set to the maximum airspeed you achieved during level flight at `FW_THR_MAX`.

#### 3rd: Pitch & Climb Rate Limits

:::warning
Do not use [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX), [FW_T_SINK_MAX](../advanced_config/parameter_reference.md#FW_T_SINK_MAX) or [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) to specify the desired climb or sink performance you would like to get from the vehicle!
Параметри визначають обмеження функціонування і повинні бути встановлені під час фази налаштування, як описано нижче.
:::

Fly in stabilized mode, apply full throttle (`FW_THR_MAX`) and slowly increase the pitch angle of the vehicle until the airspeed reaches `FW_AIRSPD_TRIM`.

- [FW_P_LIM_MAX](../advanced_config/parameter_reference.md#FW_P_LIM_MAX) - set to the pitch angle required to climb at trim airspeed when applying `FW_THR_MAX`.
- [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX) - set to the climb rate achieved during the climb at `FW_AIRSPD_TRIM`.

Fly in stabilized mode, reduce the throttle to `FW_THR_MIN` and slowly decrease the pitch angle until the vehicle reaches `FW_AIRSPD_MAX`.

- [FW_P_LIM_MIN](../advanced_config/parameter_reference.md#FW_P_LIM_MIN) - set to the pitch angle required to reach `FW_AIRSPD_MAX` at `FW_THR_MIN`.
- [FW_T_SINK_MAX](../advanced_config/parameter_reference.md#FW_T_SINK_MAX) - set to the sink rate achieved during the descent.

Fly in stabilized mode, reduce throttle to `FW_THR_MIN` and adjust the pitch angle such that the plane maintains `FW_AIRSPD_TRIM`.

- [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) - set to the sink rate achieved while maintaining `FW_AIRSPD_TRIM`.

Specify the target climb and sink rate for autonomous missions by adjusting [FW_T_CLMB_R_SP](../advanced_config/parameter_reference.md#FW_T_CLMB_R_SP) and [FW_T_SINK_R_SP](../advanced_config/parameter_reference.md#FW_T_SINK_R_SP).
Ці параметри вказують на швидкість, з якою транспортний засіб буде підніматися або сходити, щоб змінити висоту.
Furthermore, these two values define the height rate limits commanded by the user in [Altitude mode](../flight_modes_fw/altitude.md) and [Position mode](../flight_modes_fw/position.md).

### Налаштування контролю траєкторії FW (Позиція)

All path control parameters are described [here](../advanced_config/parameter_reference.md#fw-path-control).

- [NPFG_PERIOD](../advanced_config/parameter_reference.md#NPFG_PERIOD) - This is the previously called L1 distance and defines the tracking point ahead of the aircraft it's following.
  Значення 10-20 метрів працює для більшості літаків.
  Повільно скорочуйте під час налаштування до тих пір, поки відповідь не буде гострою без коливань.
  Транспортні з повільною динамікою кочення повинні мати це значення збільшене.
