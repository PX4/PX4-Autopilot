# Test MC_07 - VIO (Visual-Inertial Odometry)

## Objective

To test that external vision (VIO) works as expected

## Preflight

Disconnect all GPS / compasses and ensure vehicle is using VIO for navigation

Ensure that the drone can go into Altitude / Position flight mode while still on the ground

Ensure there are no other sources of positioning besides VIO:

- [EKF2_OF_CTRL](../advanced_config/parameter_reference.md#EKF2_OF_CTRL): `0`
- [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL): `0`
- [EKF2_EV_CTRL](../advanced_config/parameter_reference.md#EKF2_EV_CTRL): `15`
- [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG): `0`

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

## 降落

❏ Land in either Position or Altitude mode with the throttle below 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (default: see [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## 预期成果

- 当油门升高时，起飞应该是平稳的
- Drone should hold altitude in Altitude Flight mode without wandering
- Drone should hold position within 1 meter in Position Flight mode without pilot moving sticks
- 在上述任何飞行模式中都不应出现振荡
- 着陆时，直升机不应在地面上反弹
