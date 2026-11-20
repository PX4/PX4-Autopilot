# Tiltrotor VTOL Configuration

This topic covers configuration and tuning that is specific to [Tiltrotor VTOL](../frames_vtol/tiltrotor.md) vehicles.

::: warning
This topic is incomplete.
It covers setup and how the transitions work, but has little guidance on choosing parameter values for a particular vehicle.
If you have experience tuning a tiltrotor, please [contribute](../contribute/docs.md) to this page.
:::

## Firmware and Basic Settings

1. Perform the [Standard Configuration](../config/index.md), including calibration of the [airspeed sensor](../config/airspeed.md) (recommended).
1. In the [Frame setup](../config/airframe.md), select an airframe under **VTOL Tiltrotor**.
   If your vehicle is not listed, select the [Generic Quadplane VTOL Tiltrotor](../airframes/airframe_reference.md#vtol_vtol_tiltrotor_generic_quadplane_vtol_tiltrotor) or the [Generic Tiltrotor VTOL](../airframes/airframe_reference.md#vtol_vtol_tiltrotor_generic_tiltrotor_vtol).
1. Assign an RC switch for transitions, as described in [QuadPlane Configuration > Flight / Transition Mode Switch](../config_vtol/vtol_quad_configuration.md#flight-transition-mode-switch).

Before your first transition, tune the vehicle in multicopter mode and, if possible, in fixed-wing mode.
See [QuadPlane Configuration > Multirotor / Fixed-wing Tuning](../config_vtol/vtol_quad_configuration.md#multirotor-fixed-wing-tuning).

## Actuator Setup

Tiltrotor geometry and outputs are set up in [Actuator Configuration](../config/actuators.md):

- [Motor Geometry: VTOL Tiltrotor](../config/actuators.md#motor-geometry-vtol-tiltrotor): sets which tilt servo tilts each motor (`Tilted by`).
  Motors that have no tilt servo turn off in fixed-wing flight.
- [Motor Tilt Servo Geometry](../config/actuators.md#motor-tilt-servo-geometry): sets the tilt range and direction of each servo, and whether it is [used for yaw or pitch control](../config/actuators.md#tilt-servos-for-yaw-pitch-control).
- [Tilt Servo Setup](../config/actuators.md#tilt-servo-setup): sets the servo outputs, and checks the servo direction with the actuator test sliders.

### Tilt Servo Calibration

The tilt parameters ([VT_TILT_MC](../advanced_config/parameter_reference.md#VT_TILT_MC), [VT_TILT_TRANS](../advanced_config/parameter_reference.md#VT_TILT_TRANS) and [VT_TILT_FW](../advanced_config/parameter_reference.md#VT_TILT_FW)) are normalised: `0` is the servo's `Angle at min tilt` and `1` is its `Angle at max tilt`.

To calibrate the tilt servos:

1. In [Manual mode](../flight_modes_mc/manual_stabilized.md), check that the rotors point upwards.
   If they point forwards, reverse the tilt servos.
1. Adjust the servo range so that the rotors can tilt backwards past vertical, which is needed for yaw control in multicopter mode.
1. Adjust `VT_TILT_MC` so that the rotors point straight up in multicopter mode.
1. Enter `commander transition` in the [MAVLink Shell](../debug/mavlink_shell.md), and check that the rotors point straight forward.
   If they do not, adjust the servo range or `VT_TILT_FW`.

The [OMP Hobby ZMO FPV](../frames_vtol/vtol_tiltrotor_omp_hobby_zmo_fpv.md#actuator-setup) build shows these steps for a particular vehicle.

## Transitions

### Front Transition

The front transition (multicopter to fixed-wing) has two phases:

1. All rotors run, and the rotors tilt from `VT_TILT_MC` to `VT_TILT_TRANS` over [VT_F_TRANS_DUR](../advanced_config/parameter_reference.md#VT_F_TRANS_DUR) while the vehicle accelerates.
   The multicopter controller hands roll control to the fixed-wing controller as the airspeed rises from [VT_ARSP_BLEND](../advanced_config/parameter_reference.md#VT_ARSP_BLEND) to [VT_ARSP_TRANS](../advanced_config/parameter_reference.md#VT_ARSP_TRANS).
   The phase ends when the rotors have reached `VT_TILT_TRANS` and the airspeed has reached `VT_ARSP_TRANS`, after at least [VT_TRANS_MIN_TM](../advanced_config/parameter_reference.md#VT_TRANS_MIN_TM).
   Without an airspeed sensor, the airspeed condition is replaced by [VT_F_TR_OL_TM](../advanced_config/parameter_reference.md#VT_F_TR_OL_TM) having elapsed.
1. The rotors tilt the rest of the way to `VT_TILT_FW` over [VT_TRANS_P2_DUR](../advanced_config/parameter_reference.md#VT_TRANS_P2_DUR), and the vehicle is then in fixed-wing mode.

The [VT_TRANS_TIMEOUT](../advanced_config/parameter_reference.md#VT_TRANS_TIMEOUT) and airspeed checks that abort a slow transition work the same way as for a QuadPlane.
See [QuadPlane Configuration > Transition Tuning](../config_vtol/vtol_quad_configuration.md#transition-tuning) for the airspeed and timing parameters, except for `VT_F_TRANS_THR` and `VT_PSHER_SLEW`, which tiltrotors do not use.

### Back Transition

In the back transition (fixed-wing to multicopter):

1. The throttle ramps down to zero over 0.5 seconds.
1. The rotors tilt from `VT_TILT_FW` to `VT_TILT_MC` over [VT_BT_TILT_DUR](../advanced_config/parameter_reference.md#VT_BT_TILT_DUR), with the motors at idle.
1. The multicopter throttle ramps up over 0.5 seconds.

The back transition ends when the rotors have reached `VT_TILT_MC` and either the forward speed has dropped below [MPC_XY_CRUISE](../advanced_config/parameter_reference.md#MPC_XY_CRUISE) or [VT_B_TRANS_DUR](../advanced_config/parameter_reference.md#VT_B_TRANS_DUR) has elapsed.

Deceleration during the back transition is tuned as described in [Back-transition Tuning](../config_vtol/vtol_back_transition_tuning.md).

### Aborting a Transition

A multicopter request during a front transition switches the vehicle to multicopter mode immediately.

A fixed-wing request during a back transition switches the vehicle to fixed-wing mode immediately.

## Hover and Fixed-wing Flight

- [VT_FWD_THRUST_EN](../advanced_config/parameter_reference.md#VT_FWD_THRUST_EN): in hover, tilts the rotors forward to accelerate when the demanded pitch is below [VT_PITCH_MIN](../advanced_config/parameter_reference.md#VT_PITCH_MIN).
  [VT_FWD_THRUST_SC](../advanced_config/parameter_reference.md#VT_FWD_THRUST_SC) sets how strongly.
- [VT_ELEV_MC_LOCK](../advanced_config/parameter_reference.md#VT_ELEV_MC_LOCK): locks the control surfaces at their disarmed position in multicopter mode.
- [VT_FW_DIFTHR_EN](../advanced_config/parameter_reference.md#VT_FW_DIFTHR_EN): enables differential motor thrust for yaw in fixed-wing mode, scaled by [VT_FW_DIFTHR_S_Y](../advanced_config/parameter_reference.md#VT_FW_DIFTHR_S_Y).
  Tiltrotors use only the yaw bit of this parameter.

## See Also

- [VTOL Configuration](../config_vtol/index.md)
- [Back-transition Tuning](../config_vtol/vtol_back_transition_tuning.md)
- [VTOL without an Airspeed Sensor](../config_vtol/vtol_without_airspeed_sensor.md)
