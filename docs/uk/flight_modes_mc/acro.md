# Acro Режим (Мультикоптер)

<img src="../../assets/site/difficulty_hard.png" title="Hard to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;

_Acro mode_ is the RC mode for performing acrobatic maneuvers e.g. flips, rolls and loops.

Ручки крена, тангажу та риштування керують швидкістю кутової обертання навколо відповідних вісей, а керування тяги безпосередньо передається до розподілу керування. Коли стіки будуть відцентровані, апарат перестане обертатися, але залишиться у своїй поточній орієнтації (на боці, перевернутий, тощо) і рухатиметься відповідно до свого поточного імпульсу.

![MC Manual Acrobatic Flight](../../assets/flight_modes/acrobatic_mc.png)

<!-- image above incorrect: https://github.com/PX4/PX4-user_guide/issues/182 -->

## Технічний опис

Ручний режим для виконання акробатичних маневрів, наприклад, сальто, перекиди та петлі.

RC Штоки кочення/тангажу/рискання (RPY) контролюють швидкість кутового обертання навколо відповідних осей.
Передача дроселя здійснюється безпосередньо для керування розподілом.
Коли стіки будуть відцентровані, апарат перестане обертатися, але залишиться у своїй поточній орієнтації (не обов'язково рівний) і рухатиметься відповідно до свого поточного імпульсу.

Потрібен ручний ввід управління (наприклад, за допомогою пульта дистанційного керування, джойстика):

- Roll, Pitch, Yaw: Assistance from autopilot to stabilize the attitude rate.
  Положення палиці RC відображає швидкість обертання транспортного засобу в цій орієнтації.
- Дросель: Ручне керування за допомогою палиць RC.
  RC ввід передається напряму до розподілу керування.

## Відображення стіку введення

The default values for expo and rate [parameters](#parameters) are _beginner friendly_, reducing the chance that users will flip the vehicle when first trying this mode, or when using Acro mode for manual rate tuning.

Unfortunately they are poor for Acro flying though!
The following values are more reasonable for experienced users:

- [MC_ACRO_R_MAX](#MC_ACRO_R_MAX): `720` (2 turns per second)
- [MC_ACRO_P_MAX](#MC_ACRO_P_MAX): `720` (2 turns per second)
- [MC_ACRO_Y_MAX](#MC_ACRO_Y_MAX): `540` (2 turns per second)
- [MC_ACRO_EXPO](#MC_ACRO_R_MAX): `0.69`
- [MC_ACRO_EXPO_Y](#MC_ACRO_EXPO_Y): `0.69`
- [MC_ACRO_SUPEXPO](#MC_ACRO_SUPEXPO): `0.69`
- [MC_ACRO_SUPEXPOY](#MC_ACRO_SUPEXPOY): `0.7`

The roll, pitch, and yaw input stick mapping for Acro mode using the values above are shown below.
Крива дозволяє високу швидкість повороту при максимальному ввімкненні стіку для виконання акробатичних маневрів, а також зону нижчої чутливості близько до центру стіку для невеликих корекцій.

![Acro mode - default input curve](../../assets/flight_modes/acro_mc_input_curve_expo_superexpo_default.png)

This roll and pitch input stick response can be tuned using the [MC_ACRO_EXPO](#MC_ACRO_EXPO) and [MC_ACRO_SUPEXPO](#MC_ACRO_SUPEXPO) "exponential" parameters, while the yaw stick input response is tuned using [MC_ACRO_EXPO_Y](#MC_ACRO_EXPO_Y) and [MC_ACRO_SUPEXPOY](#MC_ACRO_SUPEXPOY).
`MC_ACRO_EXPO` and `MC_ACRO_EXPO_Y` tune the curve(s) between a linear and cubic curve as shown below.
`MC_ACRO_SUPEXPO` and `MC_ACRO_SUPEXPOY` allow the shape to be further tuned, modifying the width of the area of reduced sensitivity.

![Acro mode - expo - pure linear input curve](../../assets/flight_modes/acro_mc_input_curve_expo_linear.png) ![Acro mode - expo - pure cubic input curve](../../assets/flight_modes/acro_mc_input_curve_expo_cubic.png)

:::info
The mathematical relationship is:

$$\mathrm{y} = r(f \cdot x^3 + x(1-f)) (1-g)/(1-g |x|)$$

Where `f = MC_ACRO_EXPO` or `MC_ACRO_EXPO_Y`, `g = MC_ACRO_SUPEXPO` or `MC_ACRO_SUPEXPOY`, and `r` is the maximum rate.

You can experiment with the relationships graphically using the [PX4 SuperExpo calculator](https://www.desmos.com/calculator/yty5kgurmc).
:::

## Параметри

| Параметр                                                                                                                                                             | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MC_ACRO_EXPO"></a>[MC_ACRO_EXPO](../advanced_config/parameter_reference.md#MC_ACRO_EXPO)                            | Режим "експоненціальний" фактор для налаштування форми кривої вводу стіку для крену та тангажу. Values: `0` Purely linear input curve, `1` Purely cubic input curve. Default: `0`.                                                                                                                                                                            |
| <a id="MC_ACRO_EXPO_Y"></a>[MC_ACRO_EXPO_Y](../advanced_config/parameter_reference.md#MC_ACRO_EXPO_Y) | Режим "експоненціальний" фактор для налаштування форми кривої вводу стіку для рискання. Values: `0` Purely linear input curve, `1` Purely cubic input curve. Default: `0`.69.                                                                                                                                                                 |
| <a id="MC_ACRO_SUPEXPO"></a>[MC_ACRO_SUPEXPO](../advanced_config/parameter_reference.md#MC_ACRO_SUPEXPO)                   | Acro mode "SuperExpo" factor for refining stick input curve shape for the roll and pitch axes (tuned using `MC_ACRO_EXPO`). Значення: 0 Чиста функція Експо, 0.7 розумне покращення форми для інтуїтивного відчуття стіку, 0.95 дуже сильно зігнута крива впливає лише близько до максимуму. Default: `0`. |
| <a id="MC_ACRO_SUPEXPOY"></a>[MC_ACRO_SUPEXPOY](../advanced_config/parameter_reference.md#MC_ACRO_SUPEXPOY)                | Acro mode "SuperExpo" factor for refining stick input curve shape for the yaw axis (tuned using `MC_ACRO_EXPO_Y`). Values: `0` Pure Expo function, `0.7` reasonable shape enhancement for intuitive stick feel, `0.95` very strong bent input curve only near maxima have effect. Default: `0`.                                            |
| <a id="MC_ACRO_P_MAX"></a>[MC_ACRO_P_MAX](../advanced_config/parameter_reference.md#MC_ACRO_P_MAX)    | Максимальний кут нахилу тангажу. Default: `100.0` deg/s.                                                                                                                                                                                                                                                                                                                                      |
| <a id="MC_ACRO_R_MAX"></a>[MC_ACRO_R_MAX](../advanced_config/parameter_reference.md#MC_ACRO_R_MAX)    | Максимальний кут нахилу крену. Default: `100` deg/s.                                                                                                                                                                                                                                                                                                                                          |
| <a id="MC_ACRO_Y_MAX"></a>[MC_ACRO_Y_MAX](../advanced_config/parameter_reference.md#MC_ACRO_Y_MAX)    | Максимальний кут рискання. Default: `100` deg/s.                                                                                                                                                                                                                                                                                                                                              |
