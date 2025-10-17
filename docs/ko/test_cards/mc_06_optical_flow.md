# Test MC_06 - Optical Flow

## Objective

Test that optical flow works as expected

## Preflight

Disconnect all GPS / compasses and ensure vehicle is using optical flow for navigation ([setup information here](../sensor/optical_flow.md))

Ensure there are no other sources of positioning besides optical flow:

- [EKF2_OF_CTRL](../advanced_config/parameter_reference.md#EKF2_OF_CTRL): `1`
- [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL): `0`
- [EKF2_EV_CTRL](../advanced_config/parameter_reference.md#EKF2_EV_CTRL): `0`

Ensure that the drone can go into Altitude / Position mode while still on the ground

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

❏ Varying height terrain

&nbsp;&nbsp;&nbsp;&nbsp;❏ Put boxes on the ground to create varying heights in terrain

&nbsp;&nbsp;&nbsp;&nbsp;❏ Take off in position mode and fly over the boxes such that the downward facing rangefinder varies in value

&nbsp;&nbsp;&nbsp;&nbsp;❏ Do a few passes with varying amounts of time over the boxes (1-30 seconds if possible)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Drone should not raise in height when flying over boxes

## 착륙

❏ Land in either Position or Altitude mode with the throttle below 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (default: see [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## 예상 결과

- 추력을 올릴 때 서서히 이륙한다
- Drone should hold altitude in Altitude mode without wandering (over surface with many features)
- Drone should hold position within 1 meter in Position mode without pilot moving sticks
- 위에 언급한 어떤 비행 모드에서도 떨림이 나타나서는 안됨
- Drone should not raise in height when flying over boxes
- 지면에 착륙시, 콥터가 지면에서 튀면 안됨
