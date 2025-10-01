# Test MC_10 - Optical Flow / GPS Mixed

## Objective

Test that optical flow mixed with GPS works as expected

## Preflight

[Setup optical flow and GPS](../sensor/optical_flow.md)

Ensure there are no other sources of positioning besides optical flow

- [EKF2_OF_CTRL](../advanced_config/parameter_reference.md#EKF2_OF_CTRL): `1`
- [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL): `7`
- [EKF2_EV_CTRL](../advanced_config/parameter_reference.md#EKF2_EV_CTRL): `0`
- [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG): `1`
- [EKF2_HGT_REF](../advanced_config/parameter_reference.md#EKF2_HGT_REF): `1` (GPS)

Ensure that the drone can go into [Altitude](../flight_modes_mc/altitude.md) / [Position](../flight_modes_mc/position.md) mode while still on the ground

## Flight Tests

❏ [Altitude mode](../flight_modes_mc/altitude.md)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to climb/descent rate

❏ [Position mode](../flight_modes_mc/position.md)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Horizontal position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to climb/descent rate

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response set to pitch/roll/yaw rates

❏ GPS Cutout

&nbsp;&nbsp;&nbsp;&nbsp;❏ Takeoff in position mode in GPS rich environment (outdoors)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Open QGC and navigate to MAVLink Console

&nbsp;&nbsp;&nbsp;&nbsp;❏ Type `gps off` to disable GPS

&nbsp;&nbsp;&nbsp;&nbsp;❏ Drone should maintain position hold via optical flow

❏ GPS Degredation

&nbsp;&nbsp;&nbsp;&nbsp;❏ Takeoff in position mode in GPS rich environment (outdoors)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly under a metal surface (or other GPS blocking structure)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Ensure drone does not lose position hold or start drifting

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly out of metal structure to regain GPS

❏ GPS Acquisition

&nbsp;&nbsp;&nbsp;&nbsp;❏ Takeoff in position mode in non-GPS environment

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly into a GPS rich environment (outdoors)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Ensure drone acquires GPS position

## Expected Results

- Take-off should be smooth as throttle is raised
- Drone should hold position within 1 meter in Position mode without pilot moving sticks
- Drone should hold position in GPS rich environment as well as non-GPS environment
- No oscillations should present in any of the above flight modes
