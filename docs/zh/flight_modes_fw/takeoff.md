# Takeoff Mode (Fixed-Wing)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Takeoff_ flight mode causes the vehicle to take off to a specified height and then enter [Hold mode](../flight_modes_fw/takeoff.md).

Vehicles are [hand or catapult launched](#catapult-hand-launch) by default, but can also be [configured](#RWTO_TKOFF) to use a [runway takeoff](#runway-takeoff) when supported by the hardware.

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires at least a valid local position estimate (does not require a global position).
  - Flying vehicles can't switch to this mode without valid local position.
  - Flying vehicles will failsafe if they lose the position estimate.
  - Disarmed vehicles can switch to mode without valid position estimate but can't arm.
- RC control switches can be used to change flight modes.
- RC stick movement is ignored in catapult takeoff but can can be used to nudge the vehicle in runway takeoff.
- The [Failure Detector](../config/safety.md#failure-detector) will automatically stop the engines if there is a problem on takeoff.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## 技术总结

Takeoff mode (and [fixed wing mission takeoff](../flight_modes_fw/mission.md#mission-takeoff)) has two modalities: [catapult/hand-launch](#catapult-hand-launch) or [runway takeoff](#runway-takeoff) (hardware-dependent).
The mode defaults to catapult/hand launch, but can be set to runway takeoff by setting [RWTO_TKOFF](#RWTO_TKOFF) to 1.

To use _Takeoff mode_ you first switch to the mode, and then arm the vehicle.
The acceleration of hand/catapult launch triggers the motors to start.
For runway launch, motors ramp up automatically once the vehicle has been armed.

Irrespective of the modality, a flight path (starting point and takeoff course) and clearance altitude are defined:

- The starting point is the vehicle position when the takeoff mode is first entered.
- The course is set to the vehicle heading on arming
- The clearance altitude is set to [MIS_TAKEOFF_ALT](#MIS_TAKEOFF_ALT).

On takeoff, the aircraft will follow line defined by the starting point and course, climbing at the maximum climb rate ([FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX)) until reaching the clearance altitude.
Reaching the clearance altitude causes the vehicle to enter [Hold mode](../flight_modes_fw/takeoff.md).

### 参数

Parameters that affect both catapult/hand-launch and runway takeoffs:

| 参数                                                                                                                                                                         | 描述                                                                                                                                                                                            |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MIS_TAKEOFF_ALT"></a>[MIS_TAKEOFF_ALT](../advanced_config/parameter_reference.md#MIS_TAKEOFF_ALT)                         | Minimum altitude setpoint above Home that the vehicle will climb to during takeoff.                                                                                           |
| <a id="FW_TKO_AIRSPD"></a>[FW_TKO_AIRSPD](../advanced_config/parameter_reference.md#FW_TKO_AIRSPD)                               | Takeoff airspeed (is set to [FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN) if not defined by operator) |
| <a id="FW_TKO_PITCH_MIN"></a>[FW_TKO_PITCH_MIN](../advanced_config/parameter_reference.md#FW_TKO_PITCH_MIN) | This is the minimum pitch angle setpoint during the climbout phase                                                                                                                            |
| <a id="FW_T_CLMB_MAX"></a>[FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX)          | Maximum climb rate.                                                                                                                                                           |
| <a id="FW_FLAPS_TO_SCL"></a>[FW_FLAPS_TO_SCL](../advanced_config/parameter_reference.md#FW_FLAPS_TO_SCL)    | Flaps setpoint during takeoff                                                                                                                                                                 |

:::info
The vehicle always respects normal FW max/min throttle settings during takeoff ([FW_THR_MIN](../advanced_config/parameter_reference.md#FW_THR_MIN), [FW_THR_MAX](../advanced_config/parameter_reference.md#FW_THR_MAX)).
:::

<a id="hand_launch"></a>

## Catapult/Hand Launch

In _catapult/hand-launch mode_ the vehicle waits to detect launch (based on acceleration trigger).
On launch it enables the motor(s) and climbs with the maximum climb rate [FW_T_CLMB_MAX](#FW_T_CLMB_MAX) while keeping the pitch setpoint above [FW_TKO_PITCH_MIN](#FW_TKO_PITCH_MIN).
Once it reaches [MIS_TAKEOFF_ALT](#MIS_TAKEOFF_ALT) it will automatically switch to [Hold mode](../flight_modes_fw/hold.md) and loiter.

All RC stick movement is ignored during the full takeoff sequence.

To launch in this mode:

1. Arm the vehicle
2. Put the vehicle into _Takeoff mode_
3. Launch/throw the vehicle (firmly) directly into the wind.
  You can also shake the vehicle first, wait till the motor spins up and then throw it

### Parameters (Launch Detector)

The _launch detector_ is affected by the following parameters:

| 参数                                                                                                                                                                         | 描述                                                                                                          |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| <a id="FW_LAUN_DETCN_ON"></a>[FW_LAUN_DETCN_ON](../advanced_config/parameter_reference.md#FW_LAUN_DETCN_ON) | Enable automatic launch detection. If disabled motors spin up on arming already             |
| <a id="FW_LAUN_AC_THLD"></a>[FW_LAUN_AC_THLD](../advanced_config/parameter_reference.md#FW_LAUN_AC_THLD)    | Acceleration threshold (acceleration in body-forward direction must be above this value) |
| <a id="FW_LAUN_AC_T"></a>[FW_LAUN_AC_T](../advanced_config/parameter_reference.md#FW_LAUN_AC_T)             | Trigger time (acceleration must be above threshold for this amount of seconds)           |
| <a id="FW_LAUN_MOT_DEL"></a>[FW_LAUN_MOT_DEL](../advanced_config/parameter_reference.md#FW_LAUN_MOT_DEL)    | Delay from launch detection to motor spin up                                                                |

<a id="runway_launch"></a>

## Runway Takeoff

Runway takeoffs can be used by vehicles with landing gear and and steerable wheel (only).
You will first need to enable the wheel controller using the parameter [FW_W_EN](#FW_W_EN).

Vehicle should be centered and aligned with runway when takeoff is initiated.
The operator can "nudge" the vehicle while on the runway to help keeping it centered and aligned (see [RWTO_NUDGE](../advanced_config/parameter_reference.md#RWTO_NUDGE)).

The _runway takeoff mode_ has the following phases:

1. **Throttle ramp**: Throttle is ramped up within [RWTO_RAMP_TIME](../advanced_config/parameter_reference.md#RWTO_RAMP_TIME) to [RWTO_MAX_THR](../advanced_config/parameter_reference.md#RWTO_MAX_THR).
2. **Clamped to runway**: Pitch fixed, no roll and takeoff path controlled until the rotation airspeed ([RWTO_ROT_AIRSPD](../advanced_config/parameter_reference.md#RWTO_ROT_AIRSPD)) is reached. The operator is able to nudge the vehicle left/right via yaw stick.
3. **Climbout**: Increase pitch setpoint and climb to takeoff altitude. To prevent wingstrikes, the controller will keep the roll setpoint locked to 0 when close to the ground, and then gradually allow more roll while climbing. It is based on the vehicle geometry as configured in [FW_WING_SPAN](#FW_WING_SPAN) and [FW_WING_HEIGHT](#FW_WING_HEIGHT).

:::info
For a smooth takeoff, the runway wheel controller possibly needs to be tuned.
It consists of a rate controller (P-I-FF-controller with the parameters [FW_WR_P](../advanced_config/parameter_reference.md#FW_WR_P), [FW_WR_I](../advanced_config/parameter_reference.md#FW_WR_I), [FW_WR_FF](../advanced_config/parameter_reference.md#FW_WR_FF)) and an outer loop that calculates heading setpoints from course errors and can be tuned via [RWTO_NPFG_PERIOD](#RWTO_NPFG_PERIOD).
:::

### Parameters (Runway Takeoff)

Runway takeoff is affected by the following parameters:

| 参数                                                                                                                                                    | 描述                                                                                                                                                                                                                           |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RWTO_TKOFF"></a>[RWTO_TKOFF](../advanced_config/parameter_reference.md#RWTO_TKOFF)                                        | Enable runway takeoff                                                                                                                                                                                                        |
| <a id="FW_W_EN"></a>[FW_W_EN](../advanced_config/parameter_reference.md#FW_W_EN)                            | Enable wheel controller                                                                                                                                                                                                      |
| <a id="RWTO_MAX_THR"></a>[RWTO_MAX_THR](../advanced_config/parameter_reference.md#RWTO_MAX_THR)             | Max throttle during runway takeoff                                                                                                                                                                                           |
| <a id="RWTO_RAMP_TIME"></a>[RWTO_RAMP_TIME](../advanced_config/parameter_reference.md#RWTO_RAMP_TIME)       | Throttle ramp up time                                                                                                                                                                                                        |
| <a id="RWTO_ROT_AIRSPD"></a>[RWTO_ROT_AIRSPD](../advanced_config/parameter_reference.md#RWTO_ROT_AIRSPD)    | Airspeed threshold to start rotation (pitching up). If not configured by operator is set to 0.9\*FW_TKO_AIRSPD. |
| <a id="RWTO_ROT_TIME"></a>[RWTO_ROT_TIME](../advanced_config/parameter_reference.md#RWTO_ROT_TIME)          | This is the time desired to linearly ramp in takeoff pitch constraints during the takeoff rotation.                                                                                                          |
| <a id="FW_TKO_AIRSPD"></a>[FW_TKO_AIRSPD](../advanced_config/parameter_reference.md#FW_TKO_AIRSPD)          | Airspeed setpoint during the takeoff climbout phase (after rotation). If not configured by operator is set to FW_AIRSPD_MIN.    |
| <a id="RWTO_NUDGE"></a>[RWTO_NUDGE](../advanced_config/parameter_reference.md#RWTO_NUDGE)                                        | Enable wheel controller nudging while on the runway                                                                                                                                                                          |
| <a id="FW_WING_SPAN"></a>[FW_WING_SPAN](../advanced_config/parameter_reference.md#FW_WING_SPAN)             | The wingspan of the vehicle. Used to prevent wingstrikes.                                                                                                                                    |
| <a id="FW_WING_HEIGHT"></a>[FW_WING_HEIGHT](../advanced_config/parameter_reference.md#FW_WING_HEIGHT)       | The height of the wings above ground (ground clearance). Used to prevent wingstrikes.                                                                                     |
| <a id="RWTO_NPFG_PERIOD"></a>[RWTO_NPFG_PERIOD](../advanced_config/parameter_reference.md#RWTO_NPFG_PERIOD) | L1 period while steering on runway. Increase for less aggressive response to course errors.                                                                                                  |

## See Also

- [Takeoff Mode (MC)](../flight_modes_mc/takeoff.md)
- [Planning a mission takeoff](../flight_modes_fw/mission.md#mission-takeoff)

<!-- this maps to AUTO_TAKEOFF in dev -->
