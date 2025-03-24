# Altitude Mode (Fixed-Wing)

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;<img src="../../assets/site/altitude_icon.svg" title="Altitude required (e.g. Baro, Rangefinder)" width="30px" />

The _Altitude_ flight mode is the safest and easiest non-GPS manual mode.
It makes it easier for pilots to control vehicle altitude, and in particular to reach and maintain a fixed altitude.
The mode will not attempt to hold the vehicle course against wind.
Airspeed is actively controlled if an airspeed sensor is installed.

:::tip
_Altitude mode_ is similar to [Position mode](../flight_modes_fw/position.md) in that both modes level the vehicle and maintain altitude when sticks are released.
The difference is that position mode holds the actual flight path (course) steady against wind, while altitude just holds the heading.
:::

The vehicle performs a [coordinated turn](https://en.wikipedia.org/wiki/Coordinated_flight) if the roll sticks are non-zero, while the pitch stick controls the rate of ascent/descent.
The throttle determines airspeed — at 50% throttle the aircraft will hold its current altitude with a preset cruise speed.

When all sticks are released/centered (no roll, pitch, yaw, and ~50% throttle) the aircraft will return to straight, level flight (subject to wind) and keep its current altitude.
This makes it easy to recover from any problems when flying.
Roll, pitch and yaw are all angle-controlled (so it is impossible to roll over or loop the vehicle).

The yaw stick can be used to increase/reduce the yaw rate of the vehicle in turns.
If left at center the controller does the turn coordination by itself, meaning that it will apply the necessary yaw rate for the current roll angle to perform a smooth turn.

The diagram below shows the mode behaviour visually (for a [mode 2 transmitter](../getting_started/rc_transmitter_receiver.md#transmitter_modes)).

![Altitude Control FW](../../assets/flight_modes/altitude_fw.png)

## Technical Summary

Altitude mode like [Stabilized mode](../flight_modes_fw/stabilized.md) but with altitude stabilization.
Airspeed is also stabilized if an airspeed sensor is present.
The vehicle course is not maintained, and can drift due to wind.

- Centered Roll/Pitch/Yaw inputs (inside deadband):
  - Autopilot levels vehicle and maintains altitude and airspeed.
- Outside center:
  - Pitch stick controls altitude.
  - Throttle stick controls the airspeed of the aircraft if an airspeed sensor is connected. Without an airspeed sensor the vehicle will fly level at trim throttle ([FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM)), increasing or decreasing throttle as needed to climb or descend.
  - Roll stick controls roll angle. Autopilot will maintain [coordinated flight](https://en.wikipedia.org/wiki/Coordinated_flight).
  - Yaw stick adds an additional yaw rate setpoint (added to the one calculated by the autopilot to maintain coordinated flight).
    Can be used to manually change the side slip of the vehicle.
- Manual control input is required (such as RC control, joystick).
- An altitude measurement source is required (usually barometer or GPS)

## Parameters

The mode is affected by the following parameters:

| Parameter                                                                                             | Description                                                          |
| ----------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| <a id="FW_AIRSPD_MIN"></a>[FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN)    | Min airspeed. Default: 10 m/s.                                       |
| <a id="FW_AIRSPD_MAX"></a>[FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX)    | Max airspeed. Default: 20 m/s.                                       |
| <a id="FW_AIRSPD_TRIM"></a>[FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) | Cruise speed. Default: 15 m/s.                                       |
| <a id="FW_MAN_P_MAX"></a>[FW_MAN_P_MAX](../advanced_config/parameter_reference.md#FW_MAN_P_MAX)       | Max pitch setpoint in attitude stabilized mode. Default: 45 degrees. |
| <a id="FW_MAN_R_MAX"></a>[FW_MAN_R_MAX](../advanced_config/parameter_reference.md#FW_MAN_R_MAX)       | Max roll setpoint in attitude stabilized mode. Default: 45 degrees.  |
| <a id="FW_T_CLMB_R_SP"></a>[FW_T_CLMB_R_SP](../advanced_config/parameter_reference.md#FW_T_CLMB_R_SP) | Max climb rate setpoint. Default: 3 m/s.                             |
| <a id="FW_T_SINK_R_SP"></a>[FW_T_SINK_R_SP](../advanced_config/parameter_reference.md#FW_T_SINK_R_SP) | Max sink rate setpoint. Default: 2 m/s.                              |

<!--
FW notes:
FW position controller is basically 2 independent pieces
* L1 is for navigation - determines the roll and yaw needed to achieve the desired waypoint (or loiter)
* TECS is for speed and height control - determines throttle and elevator position needed to achieve the commanded altitude and airspeed
Overall that gives you an attitude setpoint (roll, pitch, yaw) and throttle which is sent off to the attitude controller
-->
