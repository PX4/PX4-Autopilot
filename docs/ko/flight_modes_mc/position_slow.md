# Position Slow Mode (Multicopter)

<Badge type="tip" text="PX4 v1.15" />

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

_Position Slow_ mode is a velocity and yaw rate limited version of the regular [Position mode](../flight_modes_mc/position.md).

The mode works in exactly the same way as _Position mode_ but with the controller stick deflection re-scaled to lower maximum velocities (and proportionally lower acceleration).
You can use it to quickly slow down the vehicle to a safe speed (if it is moving faster than the maximum velocity in the limited axis).
You can also use it to get more precision from stick input, in particular when flying close to obstacles, or to comply with regulations such as [EASA's low-speed mode/function](https://www.easa.europa.eu/en/light/topics/flying-drones-close-people).

The velocity limits can be set using parameters, from an [RC Controller](../getting_started/rc_transmitter_receiver.md) rotary knob, slider, or switch, or using MAVLink.
Limits set using an RC controller override those set by MAVLink, which in turn override those set using parameters.
The limits can only be reduced below those for normal _Position_ mode.

## Set Limits using Parameters

The maximum values for slow mode horizontal velocity, vertical velocity, and yaw rate can be set using parameters.
This approach is useful when the maximum desired speed in slow mode is fixed, and you just want to be able to quickly drop to a safer speed range (perhaps using a switch on your controller).

The table below shows the parameters used to set the maximum values for _Position slow_ mode and _Position_ mode, respectively, along with their default values.

| Axis                | Position slow mode                               | Position mode                                                                                         |
| ------------------- | ------------------------------------------------ | ----------------------------------------------------------------------------------------------------- |
| Horizontal velocity | [MC\_SLOW\_DEF\_HVEL][mc_slow_def_hvel] (3 m/s)  | [MPC\_VEL\_MANUAL][mpc_vel_manual] (10 m/s)                                                           |
| Vertical velocity   | [MC\_SLOW\_DEF\_VVEL][mc_slow_def_vvel] (1 m/s)  | [MPC\_Z\_VEL\_MAX\_UP][mpc_z_vel_max_up] (3 m/s) / [MPC\_Z\_VEL\_MAX\_DN][mpc_z_vel_max_dn] (1.5 m/s) |
| Yaw rate            | [MC\_SLOW\_DEF\_YAWR][mc_slow_def_yawr] (45 °/s) | [MPC\_MAN\_Y\_MAX][mpc_man_y_max] (150 °/s)                                                           |

From this you can see, for example, that when switching from Position mode to Position slow mode, the default maximum upward horizontal velocity is reduced from 10 m/s to 3 m/s.
If traveling faster than 3 m/s horizontally you'd be slowed to 3 m/s.

Note that the parameters are used only if limits are not provided by from RC or MAVLink.

<!-- links used in table above -->

[mpc_vel_manual]: ../advanced_config/parameter_reference.md#MPC_VEL_MANUAL
[mc_slow_def_hvel]: ../advanced_config/parameter_reference.md#MC_SLOW_DEF_HVEL
[mpc_z_vel_max_up]: ../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_UP
[mpc_z_vel_max_dn]: ../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_DN
[mc_slow_def_vvel]: ../advanced_config/parameter_reference.md#MC_SLOW_DEF_VVEL
[mpc_man_y_max]: ../advanced_config/parameter_reference.md#MPC_MAN_Y_MAX
[mc_slow_def_yawr]: ../advanced_config/parameter_reference.md#MC_SLOW_DEF_YAWR

## Set Limits using RC Control

You can map a rotary knob, slider, or switch, on a [RC Controller](../getting_started/rc_transmitter_receiver.md) to set the maximum velocity of an axis (horizontal/vertical/yaw).
This approach is useful when the appropriate slow-mode maximum values can vary while flying.

If the input control is set to its highest value the vehicle will go as fast as in _Position_ mode.
If the input is set to its lowest value, the vehicle maximum velocity is set to the value in the corresponding `MC_SLOW_MIN_` parameter (shown in the table below).
If an RC control input is mapped for an axis it has priority over all other inputs.

The table below lists each axis along with the parameter used to select which RC AUX channel corresponds to the control knob, and the parameter that sets the lowest possible "maximum value" for the axis.

| Axis                | Parameter to map auxiliary input        | Parameter for minimum value of maximum velocity |
| ------------------- | --------------------------------------- | ----------------------------------------------- |
| Horizontal velocity | [MC\_SLOW\_MAP\_HVEL][mc_slow_map_hvel] | [MC\_SLOW\_MIN\_HVEL][mc_slow_min_hvel]         |
| Vertical velocity   | [MC\_SLOW\_MAP\_VVEL][mc_slow_map_vvel] | [MC\_SLOW\_MIN\_VVEL][mc_slow_min_vvel]         |
| Yaw rate            | [MC\_SLOW\_MAP\_YAWR][mc_slow_map_yawr] | [MC\_SLOW\_MIN\_YAWR][mc_slow_min_yawr]         |

<!-- links used in table above -->

[mc_slow_map_hvel]: ../advanced_config/parameter_reference.md#MC_SLOW_MAP_HVEL
[mc_slow_min_hvel]: ../advanced_config/parameter_reference.md#MC_SLOW_MIN_HVEL
[mc_slow_map_vvel]: ../advanced_config/parameter_reference.md#MC_SLOW_MAP_VVEL
[mc_slow_min_vvel]: ../advanced_config/parameter_reference.md#MC_SLOW_MIN_VVEL
[mc_slow_map_yawr]: ../advanced_config/parameter_reference.md#MC_SLOW_MAP_YAWR
[mc_slow_min_yawr]: ../advanced_config/parameter_reference.md#MC_SLOW_MIN_YAWR

To use this approach:

1. Make sure you have a remote with an extra input and an extra RC channel to transmit it's position.
2. Map the channel which contains the knobs position as one of the 6 auxiliary passthrough inputs by setting [RC_MAP_AUXn](../advanced_config/parameter_reference.md#RC_MAP_AUX1) to the corresponding RC channel number.
3. Map that auxiliary input using the appropriate `MC_SLOW_MAP_` parameter for the axis you want it to control (see table above).

For example, if you want to map RC channel `8` to limit the horizontal velocity you could set [RC\_MAP\_AUX1](../advanced_config/parameter_reference.md#RC_MAP_AUX1) to the value `8` and [MC\_SLOW\_MAP\_HVEL][mc_slow_map_hvel] to the value `1`.
The RC input from channel 8 then sets a horizontal velocity limit between [MC\_SLOW\_MIN\_HVEL][mc_slow_min_hvel] and [MPC\_VEL\_MANUAL][mpc_vel_manual].

## Set Limits using MAVLink

You can adjust the velocity limits using the MAVLink message [SET_VELOCITY_LIMITS](https://mavlink.io/en/messages/development.html#SET_VELOCITY_LIMITS).
This approach is used primarily by automatic systems, for example to slow a vehicle when zooming a camera.

The message can set the maximum value on any of the axes by supplying a non-`NAN` limit value.
This overrides limit values set in parameters, but is ignored if the axis is mapped to an RC knob.
The value can be updated from a message at any time, and is latched until either the next message or a mode switch.

Note that PX4 does not provide velocity limit telemetry (i.e. it does not support streaming the [VELOCITY_LIMITS](https://mavlink.io/en/messages/development.html#VELOCITY_LIMITS) message).

## See Also

- [Position Slow Mode](../flight_modes_mc/position.md)
