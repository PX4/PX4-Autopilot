# Altitude Cruise Mode (Multicopter)

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;<img src="../../assets/site/altitude_icon.svg" title="Altitude required (e.g. Baro, Rangefinder)" width="30px" />

_Altitude Cruise mode_ is a _relatively_ easy-to-fly manual control mode in which roll and pitch sticks control vehicle movement in the left-right and forward-back directions (relative to the "front" of the vehicle), yaw stick controls rate of rotation over the horizontal plane, and throttle controls speed of ascent-descent.

When the sticks are released/centered the vehicle will keep the current tilt and heading angle and maintain the current _altitude_.
If moving in the horizontal plane the vehicle will accelerate until the wind resistance equals the acceleration caused by the set tilt angle.
The vehicle will then continue to move with a constant velocity (unlike for Altitude mode, in which the vehicle will eventually slow down and stop).
If the wind blows the aircraft will drift in the direction of the wind even if flying perfectly level.

:::tip
_Altitude Cruise mode_ is intended for long distance flights where the same tilt angle is kept for a long period of time. It is just like [Altitude](../flight_modes_mc/altitude.md) mode but does not go back to level tilt when the sticks are released.
:::

The diagram below shows the mode behaviour visually (for a [mode 2 transmitter](../getting_started/rc_transmitter_receiver.md#transmitter_modes)).

![Altitude Control MC - Mode2 RC Controller](../../assets/flight_modes/altitude_mc.png)

## Технічний підсумок

A manual mode that is similar to [Altitude mode](../flight_modes_mc/altitude.md) but with different interpretation of roll and pitch sticks.

- Centered sticks:
  - Roll/Pitch sticks: the current tilt is kept.
  - Yaw: the current heading is kept.
  - Throttle (~50%) holds current altitude.
- Зовнішній центр:
  - Roll/Pitch sticks control the rate of change of the tilt angle, resulting in corresponding left-right and forward-back movement. A maximum stick deflection results in a tilting rate setpoint to go from level to max tilt within 0.5 seconds.
  - Yaw stick deflection rotates the tilt angle either left or right, causing the vehicle to change course. It is _not_ causing a direct rotation around the body yaw axis like in [Acro mode](../flight_modes_mc/acro.md).
  - Ручка дроселя керує швидкістю вгору/вниз з попередньо визначеною максимальною швидкістю (та швидкістю руху в інших осях).
- Зліт:
  - Після посадки транспортний засіб злетить, якщо важіль керування газом підніметься вище 62.5% від повного діапазону (від низу).
- Manual control input is required (such as RC control, joystick) to enter this mode. Other than in all other manual modes, it's though possible to disable the manual control loss failsafe by setting the corresponding flag in [COM_RCL_EXCEPT](../advanced_config/parameter_reference.md#COM_RCL_EXCEPT). In that case the current altitude, tilt and heading are kept until the manual control link is regained or the mode is exited.
  It is highly recommended to only disable the manual control loss failsafe for this mode if there is a stable data link connection to the vehicle at all times, and to enable the data link loss failsafe through [NAV_DLL_ACT](../advanced_config/parameter_reference.md#NAV_DLL_ACT).

## Параметри

Most of the relevant parameters are already covered in the corresponding section in the [Altitude mode](../flight_modes_mc/altitude.md). Here a list of parameters of particular importance for Altitude Cruise.

| Параметр                                                                                                                                                                   | Опис                                                                                                                                                                                   |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_RCL_EXCEPT"></a>[COM_RCL_EXCEPT](../advanced_config/parameter_reference.md#COM_RCL_EXCEPT)                            | The manual control failsafe can be disabled for Altitude Cruise by setting the corresponding bit in this parameter.                                                    |
| <a id="NAV_DLL_ACT"></a>[NAV_DLL_ACT](../advanced_config/parameter_reference.md#NAV_DLL_ACT)                                     | Data link lost failsafe action. Recommended to set if the manual control failsafe is disabled to avoid fly-aways.                                      |
| <a id="MPC_MAN_TILT_MAX"></a>[MPC_MAN_TILT_MAX](../advanced_config/parameter_reference.md#MPC_MAN_TILT_MAX) | The maximum tilt angle the vehicle will go to. At max stick deflection, it will take 0.5 seconds from level flight to this tilt angle. |
