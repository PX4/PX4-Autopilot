# Test MC_10 - Optical Flow / GNSS Mixed

## Objective

Test that optical flow mixed with GNSS works as expected

## Preflight

[Setup optical flow and GNSS](../sensor/optical_flow.md)

Ensure there are no other sources of positioning besides optical flow

- [EKF2_OF_CTRL](../advanced_config/parameter_reference.md#EKF2_OF_CTRL): `1`
- [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL): `7`
- [EKF2_EV_CTRL](../advanced_config/parameter_reference.md#EKF2_EV_CTRL): `0`
- [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG): `1`
- [EKF2_HGT_REF](../advanced_config/parameter_reference.md#EKF2_HGT_REF): `1` (GNSS)

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

❏ GNSS Cutout

&nbsp;&nbsp;&nbsp;&nbsp;❏ Takeoff in position mode in GNSS rich environment (outdoors)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Open QGC and navigate to MAVLink Console

&nbsp;&nbsp;&nbsp;&nbsp;❏ Type `gps stop` to disable GNSS

&nbsp;&nbsp;&nbsp;&nbsp;❏ Drone should maintain position hold via optical flow

❏ GNSS Degradation

&nbsp;&nbsp;&nbsp;&nbsp;❏ Takeoff in position mode in GNSS rich environment (outdoors)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly under a metal surface (or other GNSS blocking structure)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Ensure drone does not lose position hold or start drifting

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly out of metal structure to regain GNSS

❏ GNSS Acquisition

&nbsp;&nbsp;&nbsp;&nbsp;❏ Takeoff in position mode in non-GNSS environment

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly into a GNSS rich environment (outdoors)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Ensure drone acquires GNSS position

## Expected Results

- Take-off should be smooth as throttle is raised
- Drone should hold position within 1 meter in Position mode without pilot moving sticks
- Drone should hold position in GNSS rich environment as well as non-GNSS environment
- No oscillations should present in any of the above flight modes
