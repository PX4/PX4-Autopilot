# Посібник з обрізання фіксованого крила

Стрічки використовуються для калібрування керуючих поверхонь в умовах обрізання (відносна швидкість повітря, густина повітря, кут атаки, конфігурація літака тощо).
Правильно обрізаний літак, що летить в умовах обтримування, буде утримувати свою позицію без потреби в будь-яких керувальних сигналах від пілота або стабілізуючого комп'ютера.

General aviation, commercial and large unmanned planes trim their control surfaces using [trim tabs](https://en.wikipedia.org/wiki/Trim_tab) while small UAVs simply add an offset to the actuator of the control surface.

The [Basic trimming](#basic-trimming) section explains the purpose of each trim parameter and how to find the correct value.
The [Advanced Trimming](#advanced-trimming) section introduces parameters that can be set to automatically adjust the trims based on the measured airspeed and flap position.

## Основна обрізка

Існують кілька параметрів, які оператор може бажати використовувати для належного обрізання літака з фіксованим крилом.
Огляд цих параметрів та їх використання показано нижче:

- [RCx_TRIM](../advanced_config/parameter_reference.md#RC1_TRIM) applies trim to the signal received from the RC transmitter.
  These parameters are set automatically during [RC calibration](../config/radio.md).
- [CA_SV_CSx_TRIM](../advanced_config/parameter_reference.md#CA_SV_CS0_TRIM) applies trim to a control surfaces channel.
  Ці засоби використовуються для точного вирівнювання керуючих поверхонь до типових кутів перед польотом.
- [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF) is the pitch angle at which your aircraft flies level at trim airspeed.
  See [Level-Flight Pitch](#level-flight-pitch) for how PX4 defines and uses it.
- [FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) is used by the rate controllers to scale their output depending on the measured airspeed.
  See [Airspeed Scaling](../flight_stack/controller_diagrams.md#airspeed-scaling) for more details.
- [TRIM_ROLL](../advanced_config/parameter_reference.md#TRIM_ROLL), [TRIM_PITCH](../advanced_config/parameter_reference.md#TRIM_PITCH) and [TRIM_YAW](../advanced_config/parameter_reference.md#TRIM_YAW) apply trim to the control signals _before_ mixing.
  For example, if you have two servos for the elevator, `TRIM_PITCH` applies trim to both of them.
  Ці використовуються, коли ваші керуючі поверхні вирівняні, але літак піднімається/кочується/повертається вгору/вниз/ліворуч/праворуч під час ручного (не стабілізованого) польоту або якщо сигнал керування має постійний зсув під час стабілізованого польоту.

Правильний порядок встановлення вищезазначених параметрів:

1. Trim the servos by physically adjusting the linkages lengths if possible and fine tune by trimming the PWM channels (use `PWM_MAIN/AUX_TRIMx`) on the bench to properly set the control surfaces to their theoretical position.
2. Fly in stabilized mode at cruise speed and set `FW_PSP_OFF` to the pitch angle that the airplane needs to fly at in order to keep constant altitude during wing-leveled flight.
   If you are using an airspeed sensor, also set the correct cruise airspeed (`FW_AIRSPD_TRIM`).
3. Look at the actuator controls in the log file (upload it to [Flight Review](https://logs.px4.io) and check the _Actuator Controls_ plot for example) and set the pitch trim (`TRIM_PITCH`).
   Встановіть це значення на середнє зміщення сигналу тану під час польоту на рівні крила.

Крок 3 можна виконати перед кроком 2, якщо ви не хочете дивитися на журнал або якщо вам зручно керувати літаком вручну.
You can then trim your remote (with the trim switches) and report the values to `TRIM_PITCH` (and remove the trims from your transmitter) or update `TRIM_PITCH` directly during flight via telemetry and QGC.

## Level-Flight Pitch

PX4 measures the fixed-wing pitch angle θ between the body x-axis and the horizon.
The body frame is fixed to the vehicle: x points forward through the nose, y along the right wing and z down.
Its orientation follows the mounting of the flight controller (see [Flight Controller Orientation](../config/flight_controller_orientation.md)), not the wing chord.
For tailsitters, the fixed-wing body frame is the multicopter body frame rotated by 90° in pitch.

![Side view of a fixed-wing aircraft pitched up: the body x-axis points forward, z down and y along the right wing out of the page; the pitch angle θ is the angle between the body x-axis and the horizon, and in level flight at trim airspeed it equals FW_PSP_OFF](../../assets/config/fw/fw_pitch_angle_definition.svg)

[FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF) is θ in level flight at trim airspeed ([FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM)).
In level flight the velocity is horizontal, so this is also the angle of attack measured from the body x-axis.
It differs from the angle of attack of the wing by the angle between the wing chord and the body x-axis.

Pitch setpoints and pitch limits in PX4 are absolute pitch angles θ, and `FW_PSP_OFF` is not added to them.
This applies for example to [FW_P_LIM_MIN](../advanced_config/parameter_reference.md#FW_P_LIM_MIN), [FW_P_LIM_MAX](../advanced_config/parameter_reference.md#FW_P_LIM_MAX), [FW_TKO_PITCH_MIN](../advanced_config/parameter_reference.md#FW_TKO_PITCH_MIN), [FW_LND_FL_PMIN](../advanced_config/parameter_reference.md#FW_LND_FL_PMIN), [FW_LND_FL_PMAX](../advanced_config/parameter_reference.md#FW_LND_FL_PMAX) and [RWTO_PSP](../advanced_config/parameter_reference.md#RWTO_PSP).

`FW_PSP_OFF` itself is used in these places:

- The altitude and airspeed controller (TECS) uses `FW_PSP_OFF` as the pitch at zero climb angle: its pitch setpoint is `FW_PSP_OFF` plus the commanded climb angle plus an integrator correction.
  The logged TECS pitch setpoint (`tecs_status.pitch_sp_rad`) is absolute, while the logged pitch integrator (`tecs_status.pitch_integ`) is a correction added to it rather than a pitch angle.
- In [Stabilized mode](../flight_modes_fw/stabilized.md), the centred pitch stick commands `FW_PSP_OFF`.
- During the front transition, standard VTOLs ramp their pitch setpoint to `FW_PSP_OFF`, and tailsitters end their pitch-down rotation at it.
- The first principle check of the [airspeed validation](../advanced_config/airspeed_validation.md) considers the vehicle to be pitching down when θ is below `FW_PSP_OFF`.

## Розширений обрізка

Оскільки знижувальний момент крену, викликаний асиметричним повітряним профілем, збільшується разом із швидкістю повітря та коли висунуті закрилки, літак потребує повторного налаштування відповідно до поточно виміряної швидкості повітря та положення закрилків.
Для цієї мети можна визначити білінійну криву функції швидкості повітря та функції інкременту тримання кута за допомогою параметрів стану закріплення (див. рисунок нижче):

- [FW_DTRIM\_\[R/P/Y\]\_\[VMIN/VMAX\]](../advanced_config/parameter_reference.md#FW_DTRIM_R_VMIN) are the roll/pitch/yaw trim value added to `TRIM_ROLL/PITCH/YAW` at min/max airspeed (defined by [FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN) and [FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX)).
- [CA_SV_CSx_FLAP](../advanced_config/parameter_reference.md#CA_SV_CS0_FLAP) and [CA_SV_CSx_SPOIL](../advanced_config/parameter_reference.md#CA_SV_CS0_SPOIL) are the trimming values that are applied to these control surfaces if the flaps or the spoilers are fully deployed, respectively.

![Dtrim Curve](../../assets/config/fw/fixedwing_dtrim.png)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/15AbscUF1kRdWMh8ONcCRu6QBwGbqVGfl/view?usp=sharing
Request access from dev team. -->

Ідеально симетрична конструкція фюзеляжу вимагала б лише інкрементів відкалібрування крена, але оскільки реальний фюзеляж ніколи не є ідеально симетричним, інкременти відкалібрування крена та курсу іноді також потрібні.
