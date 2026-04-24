# Test FW_02 - Full Autonomous

## Objective

To test the auto modes such as Mission, Takeoff, Hold, and RTL for fixed wing vehicles.

## Preflight

Plan a mission on the ground. Ensure the mission has:

- Takeoff as first waypoint
- Changes in altitude throughout the mission
- Last waypoint is an RTL
- Duration of 1 to 2 minutes

## Flight Tests

❏ Takeoff

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage Takeoff mode (hand launch or runway)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle should climb to takeoff altitude

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle should hold/loiter after reaching takeoff altitude

❏ Mission

&nbsp;&nbsp;&nbsp;&nbsp;❏ Auto takeoff (hand launch or runway)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Verify changes in altitude throughout the mission

&nbsp;&nbsp;&nbsp;&nbsp;❏ Verify Mission Ends in RTL

&nbsp;&nbsp;&nbsp;&nbsp;❏ Duration of 1 to 2 minutes

&nbsp;&nbsp;&nbsp;&nbsp;❏ Auto land or hold at end

❏ Hold

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage Hold mode during flight

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle should orbit at current position and altitude

&nbsp;&nbsp;&nbsp;&nbsp;❏ Orbit radius and direction should match parameters

❏ RTL

&nbsp;&nbsp;&nbsp;&nbsp;❏ Arm and takeoff in any manual mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly out ~200m from start point

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage Return mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle should climb to RTL altitude if below it

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle should return to home and hold or land

## Expected Results

- Mission should upload on first attempt
- Vehicle should automatically takeoff upon engaging Auto
- Waypoint tracking should be smooth with appropriate turn radius
- Vehicle should adjust height to RTL altitude before returning home
- Landing approach should be stable (if auto-land is configured)
