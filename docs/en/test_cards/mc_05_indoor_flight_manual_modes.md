# Test MC_05 - Indoor Flight (Manual Modes)

## When to Use This Test Card

- New build maiden flight
- When required to replicate an issue in a confined area
- Experimental builds that might have stability issues
- Testing hardware that has been replaced and/or modified

## Arm and Take-off

❏ Set flight mode to stabilize and Arm

❏ Take-off by raising the throttle

## Flight

❏ Stabilized

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response 1:1

❏ Altitude

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

## Landing

❏ Land in either Stabilized or Altitude mode with the throttle below 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## Expected Results

- Take-off should be smooth as throttle is raised
- No oscillations should present in any of the above flight modes
- Upon landing, copter should not bounce on the ground
