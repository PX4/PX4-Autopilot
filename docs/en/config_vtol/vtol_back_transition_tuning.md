# VTOL Back-transition Tuning

When a VTOL performs a back-transition (transition from fixed-wing mode to multicopter) it needs to slow down before the multicopter can take proper control.
To help with braking, the controller will pitch up the vehicle if the current deceleration is below what is set in expected deceleration ([VT_B_DEC_MSS](../advanced_config/parameter_reference.md#VT_B_DEC_MSS)).
The response of this deceleration controller can be tuned through a `I` gain: [VT_B_DEC_I](../advanced_config/parameter_reference.md#VT_B_DEC_I).
Increasing the `I` will result in more aggressive pitch-up to achieve the configured deceleration setting.

The vehicle will consider the back-transition complete when the horizontal speed has reached multicopter cruise speed ([MPC_XY_CRUISE](../advanced_config/parameter_reference.md#MPC_XY_CRUISE)) or when the back-transition duration ([VT_B_TRANS_DUR](../advanced_config/parameter_reference.md#VT_B_TRANS_DUR)) has passed (whichever comes first).

## Setting expected deceleration

When flying missions that make use of a [VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND) waypoint, the autopilot will attempt to calculate the proper distance at which to initiate the back-transition. It does this by looking at the current velocity (comparable to ground speed) and the expected deceleration.
To get the vehicle to come out of back-transition very close to its landing point you can tune the expected deceleration ([VT_B_DEC_MSS](../advanced_config/parameter_reference.md#VT_B_DEC_MSS)) parameter.
Make sure you have a large enough back-transition duration to allow the vehicle to reach its intended position before this timeout kicks in.

## Back-transition duration

Setting a high back-transition time ([VT_B_TRANS_DUR](../advanced_config/parameter_reference.md#VT_B_TRANS_DUR)) will give the vehicle more time to slow down.
During this period the VTOL will shut down its fixed-wing motor and slowly ramp up its MC motors while gliding.
The higher this time is set the longer the vehicle will glide in an attempt to slow down. The caveat of this behavior is that the vehicle will only control altitude and not position during this period, so some drift can occur.
