# 시험 MC_01 - 수동 모드

## Objective

To test that manual flight modes work as expected

## Preflight

Ensure that the drone can go into Stabilized, Altitude, Position mode while still on the ground

## Flight Tests

❏ 안정화 상태

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response 1:1

❏ 고도

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

❏ 위치

&nbsp;&nbsp;&nbsp;&nbsp;❏ Horizontal position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response set to Pitch/Roll/Yaw rates

❏ Landing

&nbsp;&nbsp;&nbsp;&nbsp;❏ Land in Position mode with the throttle below 40%

&nbsp;&nbsp;&nbsp;&nbsp;❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## 예상 결과

- 추력을 올릴 때 서서히 이륙한다
- 위에 언급한 어떤 비행 모드에서도 떨림이 나타나서는 안됨
- 지면에 착륙시, 콥터가 지면에서 튀면 안됨
