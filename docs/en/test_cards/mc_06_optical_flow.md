# Test MC_06 - Optical Flow

## Objective

To test that optical flow / external vision work as expected

## Preflight

Disconnect all GPS / compasses and ensure vehicle is using optical flow for navigation

Ensure that the drone can go into Altitude / Position mode while still on the ground

## Flight Tests

❏ Altitude

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to climb/descent rate

❏ Position

&nbsp;&nbsp;&nbsp;&nbsp;❏ Horizontal position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to climb/descent rate

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response set to pitch/roll/yaw rates

## Landing

❏ Land in either Position or Altitude mode with the throttle below 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## Expected Results

- Take-off should be smooth as throttle is raised
- No oscillations should present in any of the above flight modes
- Upon landing, copter should not bounce on the ground
