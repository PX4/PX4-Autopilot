# Test MC_06 - Optical Flow

## Objective

To test that optical flow / external vision work as expected

## Preflight

Disconnect all GPS / compasses and ensure vehicle is using optical flow for navigation

Ensure that the drone can go into Altitude / Position flight mode while still on the ground

## Flight Tests

❏ Altitude flight mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to climb/descent rate

❏ Position flight mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Horizontal position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to climb/descent rate

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response set to pitch/roll/yaw rates

## 착륙

❏ Land in either Position or Altitude mode with the throttle below 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (default: see [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## 예상 결과

- 추력을 올릴 때 서서히 이륙한다
- 위에 언급한 어떤 비행 모드에서도 떨림이 나타나서는 안됨
- 지면에 착륙시, 콥터가 지면에서 튀면 안됨
