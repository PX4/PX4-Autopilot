# Land Mode (Fixed-Wing)

<img src="../../assets/site/position_fixed.svg" title="Position estimate required (e.g. GPS)" width="30px" />

The _Land_ flight mode causes the vehicle to descend at the position where the mode was engaged, following a circular path until touchdown.
After landing, vehicles will disarm after a short timeout (by default).

:::warning
Fixed-wing _land mode_ should only be used in an **emergency**!
The vehicle will descend around the current location irrespective of the suitability of the underlying terrain, and touch down while following a circular flight path.

Where possible, instead use [Return mode](../flight_modes_fw/return.md) with a predefined [Fixed-wing mission landing](../flight_modes_fw/mission.md#mission-landing).
:::

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires at least a valid local position estimate (does not require a global position).
  - Flying vehicles can't switch to this mode without valid local position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode prevents arming (vehicle must be armed when switching to this mode).
- 遥控开关可以在任何无人机上更改飞行模式。
- RC stick movement is ignored.
- The mode can be triggered using the [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) MAVLink command, or by explicitly switching to Land mode.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## 技术总结

Land mode causes the vehicle follow a descending circular path (corkscrew) until touchdown.

When the mode is engaged, the vehicle starts to loiter around the current vehicle position with loiter radius [NAV_LOITER_RAD](#NAV_LOITER_RAD) and begins to descend with a constant descent speed.
The descent speed is calculated using [FW_LND_ANG](#FW_LND_ANG) and the set landing airspeed [FW_LND_AIRSPD](#FW_LND_AIRSPD).
The vehicle will flare if configured to do so (see [Flaring](../flight_modes_fw/mission.md#flaring-roll-out)), and otherwise proceed circling with the constant descent rate until landing is detected.

[Manual nudging](../flight_modes_fw/mission.md#automatic-abort) and [automatic land abort](../flight_modes_fw/mission.md#nudging) are not available in land mode.

### 参数

Land mode behaviour can be configured using the parameters below.

| 参数                                                                                                                                              | 描述                                                                                           |
| ----------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| <a id="NAV_LOITER_RAD"></a>[NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_LOITER_RAD) | The loiter radius that the controller tracks for the whole landing sequence. |
| <a id="FW_LND_ANG"></a>[FW_LND_ANG](../advanced_config/parameter_reference.md#FW_LND_ANG)             | The flight path angle setpoint.                                              |
| <a id="FW_LND_AIRSPD"></a>[FW_LND_AIRSPD](../advanced_config/parameter_reference.md#FW_LND_AIRSPD)    | The airspeed setpoint.                                                       |

## See Also

- [Land Mode (MC)](../flight_modes_mc/land.md)
- [Land Mode (VTOL)](../flight_modes_vtol/land.md)
