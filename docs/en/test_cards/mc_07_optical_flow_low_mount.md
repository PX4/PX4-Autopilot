# Test MC_07 - Optical Flow Low Sensor

## Objective

Test that optical flow works as expected with a low mounted optical flow sensor

## Preflight

Ensure that the drone's optical flow sensor is mounted more than an inch off of the ground

Ensure that [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN) is tuned correctly for landing

Disconnect all GPS / compasses and ensure vehicle is using optical flow for navigation
([Setup Information here](../sensor/optical_flow.md))

Ensure there are no other sources of positioning besides optical flow

- [EKF2_OF_CTRL](../advanced_config/parameter_reference.md#EKF2_OF_CTRL): `1`
- [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL): `0`
- [EKF2_EV_CTRL](../advanced_config/parameter_reference.md#EKF2_EV_CTRL): `0`

Ensure that the drone can go into [Position mode](../flight_modes_mc/position.md) while still on the ground

## Flight Tests

❏ [Position mode](../flight_modes_mc/position.md)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Horizontal position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to climb/descent rate

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response set to pitch/roll/yaw rates

## Landing

❏ Land in Position mode with the throttle below 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (default: see [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## Expected Results

- Take-off should be smooth as throttle is raised
- Drone should stay in Position mode, NOT fall into [altitude](../flight_modes_mc/altitude.md) mode
