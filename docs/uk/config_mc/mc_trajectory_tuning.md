# Налаштування заданого пункту для багатороторного вертольота (Генератор Траєкторії)

This document provides an overview of the multicopter tuning parameters that change the _user experience_: how fast the vehicle reacts to stick movements or direction changes in missions, the maximum allowed velocity, etc.

In other words, this topic explains how to tune the parameters that affect the value of a _desired setpoint_ rather than those that affect how well the vehicle _tracks_ the setpoint).

Алгоритм, який генерує ці точки встановлення, називається "генератор траєкторії".

:::warning
This guide is for advanced users/experts.
:::

:::tip
Follow the instructions in the [Multicopter PID Tuning Guide](../config_mc/pid_tuning_guide_multicopter.md) _before_ doing any of the tuning described here.
Не використовуйте ці параметри настройки для виправлення поганого відстеження або вібрації!
:::

## Загальний огляд

The input to the P/PID controller is a _desired setpoint_ that the vehicle should attempt to track.
[PID Tuning](../config_mc/pid_tuning_guide_multicopter.md) ("Lower level tuning") aims to reduce the error between the desired setpoint and the estimate of the vehicle state.

The _desired setpoint_ passed to the P/PID controller is itself calculated from a _demanded setpoint_ based on a stick position (in RC modes) or from a mission command.
Вимагане установлення може дуже швидко змінюватися (наприклад, якщо користувач переміщує палицю від нуля до максимального значення як "крок").
Характеристики польоту транспортного засобу кращі, якщо відповідні бажані встановлені значення змінюються як "конус".

_Setpoint value tuning_ ("higher level tuning") is used to specify the mapping between the _demanded_ and the _desired_ setpoints - i.e. defining the "ramp" at which the desired setpoint follows the demanded setpoint.

:::tip
Poorly tuned [P/PID Gains](../config_mc/pid_tuning_guide_multicopter.md) can lead to instability.
Poorly tuned _setpoint values_ cannot result in instability, but may result in either very jerky or very unresponsive reactions to setpoint changes.
:::

<a id="modes"></a>

## Підтримка траєкторій режимів польоту

[Mission mode](../flight_modes_mc/mission.md) used the [Jerk-limited](../config_mc/mc_jerk_limited_type_trajectory.md) trajectory all the time.

[Position mode](../flight_modes_mc/position.md) supports the [implementations](#position-mode-implementations) listed below.
It uses the acceleration based mapping by default; other types can be set using [MPC_POS_MODE](../advanced_config/parameter_reference.md#MPC_POS_MODE).

[Altitude mode](../flight_modes_mc/altitude.md) similarly supports the [implementations](#altitude-mode-implementations) selected by [MPC_POS_MODE](../advanced_config/parameter_reference.md#MPC_POS_MODE), but _only_ for smoothing the vertical component (i.e. when controlling the altitude).

Жоден інший режим не підтримує налаштування траєкторії.

## Реалізації режиму позиціонування

The following list provides an _overview_ of the different implementations of how the stick input is interpreted and turned into trajectory setpoints:

- Заснований на прискоренні (за замовчуванням)
  - Горизонтальний вхід палиці відображений на встановлених точках прискорення.
  - Інтуїтивне відчуття палиці, оскільки це схоже на те, що ти тягнеш транспортний засіб.
  - Неочікувані зміни нахилу при досягненні швидкості руху.
  - Вертикальний вхід палиці відображений з обмеженим ривком траєкторії.
  - Set in position mode using `MPC_POS_MODE=Acceleration based`.
- [Jerk-limited](../config_mc/mc_jerk_limited_type_trajectory.md)
  - Використовується, коли потрібен плавний рух (наприклад: зйомка, картографування, вантаж).
  - Генерує симетричні плавні S-криві, де обмеження різкості та прискорення завжди гарантовані.
  - Можливо, не підходить для транспортних засобів / випадків використання, які вимагають швидкої відповіді - наприклад, гонщицькі квадрокоптери.
  - Set in position mode using `MPC_POS_MODE=Smoothed velocity`.
- **Simple position control**
  - Палиці безпосередньо відображаються на встановлені точки швидкості без згладжування.
  - Корисно для налаштування контролю швидкості.
  - Set in position mode using `MPC_POS_MODE=Direct velocity`.

## Реалізації режиму висоти

Analogously to [position mode implementations](#position-mode-implementations) these are the implementations for interpreting vertical stick input:

- [Jerk-limited](../config_mc/mc_jerk_limited_type_trajectory.md)
  - Згладжений вертикальний вхід.
  - Set in altitude mode with `MPC_POS_MODE` Smoothed velocity or Acceleration based.
- **Simple altitude control**
  - Незгладжений вертикальний вхід.
  - Set in altitude mode only when using `MPC_POS_MODE=Direct velocity`.
