# Hold Mode (Fixed-Wing)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Hold_ flight mode causes the vehicle to loiter (circle) around its current GPS position and maintain its current altitude.

:::tip
_Hold mode_ can be used to pause a mission or to help you regain control of a vehicle in an emergency.
It is usually activated with a pre-programmed switch.
:::

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
  - Disarmed vehicles can switch to mode without valid position estimate but can't arm.
- Mode requires wind and flight time are within allowed limits (specified via parameters).
- RC 제어 스위치는 기체의 비행 모드를 변경할 수 있습니다.
- RC stick movement is ignored.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Technical Summary

The aircraft circles around the GPS hold position at the current altitude.
The vehicle will first ascend to [NAV_MIN_LTR_ALT](#NAV_MIN_LTR_ALT) if the mode is engaged below this altitude.

RC stick movement is ignored.

### 매개변수

Hold mode behaviour can be configured using the parameters below.

| 매개변수                                                                                                                                                                    | 설명                                                                                                                                               |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| [NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_LOITER_RAD)                                                    | The radius of the loiter circle.                                                                                                 |
| <a id="NAV_MIN_LTR_ALT"></a>[NAV_MIN_LTR_ALT](../advanced_config/parameter_reference.md#NAV_MIN_LTR_ALT) | Minimum height for loiter mode (vehicle will ascend to this altitude if mode is engaged at a lower altitude). |

## See Also

[Hold Mode (MC)](../flight_modes_mc/hold.md)

<!-- this maps to AUTO_LOITER in flight mode state machine -->
