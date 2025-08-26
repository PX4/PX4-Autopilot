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

## Посадка

❏ Land in either Position or Altitude mode with the throttle below 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (default: see [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## Очікувані результати

- Зліт повинен бути плавним, коли газ піднято
- Немає коливання в жодному з перерахованих режимів польоту
- Після посадки, коптер не повинен підскакувати на землі
