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
- [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF) applies an offset to the pitch setpoint.
  Це використовується для встановлення кута атаки, при якому ваш літак повинен летіти з крейсерською швидкістю.
- [FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) is used by the rate controllers to scale their output depending on the measured airspeed.
  See [Airspeed Scaling](../flight_stack/controller_diagrams.md#airspeed-scaling) for more details.
- [TRIM_ROLL](../advanced_config/parameter_reference.md#TRIM_ROLL), [TRIM_PITCH](../advanced_config/parameter_reference.md#TRIM_PITCH) and [TRIM_YAW](../advanced_config/parameter_reference.md#TRIM_YAW) apply trim to the control signals _before_ mixing.
  For example, if you have two servos for the elevator, `TRIM_PITCH` applies trim to both of them.
  Ці використовуються, коли ваші керуючі поверхні вирівняні, але літак піднімається/кочується/повертається вгору/вниз/ліворуч/праворуч під час ручного (не стабілізованого) польоту або якщо сигнал керування має постійний зсув під час стабілізованого польоту.

Правильний порядок встановлення вищезазначених параметрів:

1. Trim the servos by physically adjusting the linkages lengths if possible and fine tune by trimming the PWM channels (use `PWM_MAIN/AUX_TRIMx`) on the bench to properly set the control surfaces to their theoretical position.
2. Fly in stabilized mode at cruise speed and set the pitch setpoint offset (`FW_PSP_OFF`) to desired angle of attack.
  Необхідний кут атаки при крейсерській швидкості відповідає куту крена, який потрібно літаку летіти, щоб утримати постійну висоту під час польоту з вирівняним крилом.
  If you are using an airspeed sensor, also set the correct cruise airspeed (`FW_AIRSPD_TRIM`).
3. Look at the actuator controls in the log file (upload it to [Flight Review](https://logs.px4.io) and check the _Actuator Controls_ plot for example) and set the pitch trim (`TRIM_PITCH`).
  Встановіть це значення на середнє зміщення сигналу тану під час польоту на рівні крила.

Крок 3 можна виконати перед кроком 2, якщо ви не хочете дивитися на журнал або якщо вам зручно керувати літаком вручну.
You can then trim your remote (with the trim switches) and report the values to `TRIM_PITCH` (and remove the trims from your transmitter) or update `TRIM_PITCH` directly during flight via telemetry and QGC.

## Розширений обрізка

Оскільки знижувальний момент крену, викликаний асиметричним повітряним профілем, збільшується разом із швидкістю повітря та коли висунуті закрилки, літак потребує повторного налаштування відповідно до поточно виміряної швидкості повітря та положення закрилків.
Для цієї мети можна визначити білінійну криву функції швидкості повітря та функції інкременту тримання кута за допомогою параметрів стану закріплення (див. рисунок нижче):

- [FW_DTRIM\_\[R/P/Y\]\_\[VMIN/VMAX\]](../advanced_config/parameter_reference.md#FW_DTRIM_R_VMIN) are the roll/pitch/yaw trim value added to `TRIM_ROLL/PITCH/YAW` at min/max airspeed (defined by [FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN) and [FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX)).
- [CA_SV_CSx_FLAP](../advanced_config/parameter_reference.md#CA_SV_CS0_FLAP) and [CA_SV_CSx_SPOIL](../advanced_config/parameter_reference.md#CA_SV_CS0_SPOIL) are the trimming values that are applied to these control surfaces if the flaps or the spoilers are fully deployed, respectively.

![Dtrim Curve](../../assets/config/fw/fixedwing_dtrim.png)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/15AbscUF1kRdWMh8ONcCRu6QBwGbqVGfl/view?usp=sharing
Request access from dev team. -->

Ідеально симетрична конструкція фюзеляжу вимагала б лише інкрементів відкалібрування крена, але оскільки реальний фюзеляж ніколи не є ідеально симетричним, інкременти відкалібрування крена та курсу іноді також потрібні.
