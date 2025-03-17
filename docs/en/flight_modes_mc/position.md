# Position Mode (Multicopter)

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

_Position_ is an easy-to-fly RC mode in which roll and pitch sticks control acceleration over ground in the vehicle's left-right and forward-back directions (similar to a car's accelerator pedal), and throttle controls speed of ascent-descent.
When the sticks are released/centered the vehicle will actively brake, level, and be locked to a position in 3D space — compensating for wind and other forces.
With full stick deflection the vehicle accelerates initially with [MPC_ACC_HOR_MAX](#MPC_ACC_HOR_MAX) ramping down until it reaches the final velocity [MPC_VEL_MANUAL](#MPC_VEL_MANUAL).

:::tip
Position mode is the safest manual mode for new fliers.
Unlike [Altitude](../flight_modes_mc/altitude.md) and [Stabilized](../flight_modes_mc/manual_stabilized.md) modes the vehicle will stop when the sticks are centered rather than continuing until slowed by wind resistance.
:::

The diagram below shows the mode behaviour visually (for a mode 2 transmitter).

![MC Position Mode](../../assets/flight_modes/position_mc.png)

### Landing

Landing in this mode is easy:

1. Position the drone horizontally above the landing spot using the roll and pitch stick.
1. Let go of the roll and pitch stick and give it enough time to come to a complete stop.
1. Pull the throttle stick down gently until the vehicle touches the ground.
1. Pull the throttle stick all the way down to facilitate and accelerate land detection.
1. The vehicle will lower propeller thrust, detect the ground and [automatically disarm](../advanced_config/prearm_arm_disarm.md#auto-disarming) (by default).

:::warning
While very rare on a well calibrated vehicle, sometimes there may be problems with landing.

- If the vehicle does not stop moving horizontally:
  - You can still land under control in [Altitude mode](../flight_modes_mc/altitude.md).
    The approach is the same as above, except that you must manually ensure that the vehicle stays above the landing spot using the roll and pitch stick.
  - After landing check GPS and magnetometer orientation, calibration.
- If the vehicle does not detect the ground/landing and disarm:
  - After the vehicle is on the ground switch to [Stabilized mode](../flight_modes_mc/manual_stabilized.md) keeping the throttle stick low, and manually disarm using a gesture or other command.
    Alternatively you can also use the kill switch when the vehicle is already on the ground.

:::

## Technical Summary

RC mode where roll, pitch, throttle (RPT) sticks control movement in corresponding axes/directions.
Centered sticks level vehicle and hold it to fixed altitude and position against wind.

- Centered roll, pitch, throttle sticks (within RC deadzone [MPC_HOLD_DZ](../advanced_config/parameter_reference.md#MPC_HOLD_DZ)) hold x, y, z position steady against any disturbance like wind.
- Outside center:
  - Roll/Pitch sticks control horizontal acceleration over ground in the vehicle's left-right and forward-back directions (respectively).
  - Throttle stick controls speed of ascent-descent.
  - Yaw stick controls rate of angular rotation above the horizontal plane.
- Takeoff:
  - When landed, the vehicle will take off if the throttle stick is raised above 62.5% percent (of the full range from bottom).
- Global position estimate is required.
- Manual control input is required (such as RC control, joystick). 
  - Roll, Pitch, Throttle: Assistance from autopilot to hold position against wind. 
  - Yaw: Assistance from autopilot to stabilize the attitude rate. 
    Position of RC stick maps to the rate of rotation of vehicle in that orientation. 

### Parameters

All the parameters in the [Multicopter Position Control](../advanced_config/parameter_reference.md#multicopter-position-control) group are relevant. A few parameters of particular note are listed below.

| Parameter                                                                                                   | Description                                                                                                                                                                                                                                                                                                  |
| ----------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="MPC_HOLD_DZ"></a>[MPC_HOLD_DZ](../advanced_config/parameter_reference.md#MPC_HOLD_DZ)                | Deadzone of sticks where position hold is enabled. Default: 0.1 (10% of full stick range).                                                                                                                                                                                                                   |
| <a id="MPC_Z_VEL_MAX_UP"></a>[MPC_Z_VEL_MAX_UP](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_UP) | Maximum vertical ascent velocity. Default: 3 m/s.                                                                                                                                                                                                                                                            |
| <a id="MPC_Z_VEL_MAX_DN"></a>[MPC_Z_VEL_MAX_DN](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_DN) | Maximum vertical descent velocity. Default: 1 m/s.                                                                                                                                                                                                                                                           |
| <a id="MPC_LAND_ALT1"></a>[MPC_LAND_ALT1](../advanced_config/parameter_reference.md#MPC_LAND_ALT1)          | Altitude for triggering first phase of slow landing. Below this altitude descending velocity gets limited to a value between [MPC_Z_VEL_MAX_DN](#MPC_Z_VEL_MAX_DN) (or `MPC_Z_V_AUTO_DN`) and [MPC_LAND_SPEED](#MPC_LAND_SPEED). Value needs to be higher than [MPC_LAND_ALT2](#MPC_LAND_ALT2). Default 10m. |
| <a id="MPC_LAND_ALT2"></a>[MPC_LAND_ALT2](../advanced_config/parameter_reference.md#MPC_LAND_ALT2)          | Altitude for second phase of slow landing. Below this altitude descending velocity gets limited to [`MPC_LAND_SPEED`](#MPC_LAND_SPEED). Value needs to be lower than "MPC_LAND_ALT1". Default 5m.                                                                                                            |
| <a id="RCX_DZ"></a>`RCX_DZ`                                                                                 | RC dead zone for channel X. The value of X for throttle will depend on the value of [RC_MAP_THROTTLE](../advanced_config/parameter_reference.md#RC_MAP_THROTTLE). For example, if the throttle is channel 4 then [RC4_DZ](../advanced_config/parameter_reference.md#RC4_DZ) specifies the deadzone.          |
| <a id="MPC_xxx"></a>`MPC_XXXX`                                                                              | Most of the MPC_xxx parameters affect flight behaviour in this mode (at least to some extent). For example, [MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) defines the thrust at which a vehicle will hover.                                                                       |
| <a id="MPC_POS_MODE"></a>[MPC_POS_MODE](../advanced_config/parameter_reference.md#MPC_POS_MODE)             | Stick input to movement translation strategy. From PX4 v1.12 the default (`Acceleration based`) is that stick position controls acceleration (in a similar way to a car accelerator pedal). Other options allow stick deflection to directly control speed over ground, with and without smoothing and acceleration limits.     |
| <a id="MPC_ACC_HOR_MAX"></a>[MPC_ACC_HOR_MAX](../advanced_config/parameter_reference.md#MPC_ACC_HOR_MAX)    | Maximum horizontal acceleration.                                                                                                                                                                                                                                                                             |
| <a id="MPC_VEL_MANUAL"></a>[MPC_VEL_MANUAL](../advanced_config/parameter_reference.md#MPC_VEL_MANUAL)       | Maximum horizontal velocity.                                                                                                                                                                                                                                                                                 |
| <a id="MPC_LAND_SPEED"></a>[MPC_LAND_SPEED](../advanced_config/parameter_reference.md#MPC_LAND_SPEED)       | Landing descend rate. Default 0.7 m/s.                                                                                                                                                                                                                                                                       |

## Additional Information

### Position Loss/Safety

Position mode is dependent on having an acceptable position estimate.
If the estimate falls below acceptable levels, for example due to GPS loss, this may trigger a [Position (GPS) Loss Failsafe](../config/safety.md#position-gnss-loss-failsafe).
Depending on configuration, whether you have a remote control, and whether there is an adequate altitude estimate, PX4 may switch to altitude mode, manual mode, land mode or terminate.

## See Also

- [Position Slow Mode](../flight_modes_mc/position_slow.md)
