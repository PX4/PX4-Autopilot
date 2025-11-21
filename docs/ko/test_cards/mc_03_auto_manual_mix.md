# 시험 MC_03 - 자동 / 수동 혼합

## Objective

To test switching between various modes

## Preflight

- Takeoff as first waypoint
- Changes in Altitude throughout the mission
- Last waypoint is a NOT RTL, but a normal waypoint
- Duration of 5 to 6 minutes

## Flight Tests

❏ Position + Mission

&nbsp;&nbsp;&nbsp;&nbsp;❏ Arm and take-off in Position mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage Auto

&nbsp;&nbsp;&nbsp;&nbsp;❏ Observe tracking and cornering

&nbsp;&nbsp;&nbsp;&nbsp;❏ Once mission has completed, switch back to Position mode

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;❏ Horizontal position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response set to Pitch/Roll/Yaw rates

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage RTL

&nbsp;&nbsp;&nbsp;&nbsp;❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: COM_DISARM_LAND)

❏ Mission + Controller interruption

&nbsp;&nbsp;&nbsp;&nbsp;❏ Arm and take-off in Position mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage Auto

&nbsp;&nbsp;&nbsp;&nbsp;❏ Observe tracking and cornering

&nbsp;&nbsp;&nbsp;&nbsp;❏ Before the last waypoint mission is reached, move the control sticks and ensure the vehicle goes into Position flight mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Manually move the drone over the landing zone

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage Land mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: COM_DISARM_LAND)

## 예상 결과

- 추력을 올릴 때 서서히 이륙한다
- 위에 언급한 어떤 비행 모드에서도 떨림이 나타나서는 안됨
- When moving the control sticks the drone goes into Position flight mode
- 지면에 착륙시, 콥터가 지면에서 튀면 안됨
