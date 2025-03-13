# Flight Modes (Multicopter)

Flight modes provide autopilot support to make it easier to manually fly the vehicle, to automate common tasks such as takeoff and landing, to execute autonomous missions, or to defer flight control to an external system.

This topic provides an overview of the available flight modes for multicopters and helicopters.

## Overview

Flight Modes are either _manual_ or _autonomous_.
Manual modes provide different levels of autopilot support when flying manually (using RC control sticks or a joystick), while _autonomous_ modes can be fully controlled by the autopilot.

Manual-Easy:

- [Position mode](../flight_modes_mc/position.md) — Easiest and safest manual mode for vehicles that have a position fix/GPS.
  The roll and pitch sticks control _acceleration_ over ground in the vehicle's forward-back and left-right directions (similar to a car's accelerator pedal), the yaw stick controls horizontal rotation, and the throttle controls speed of ascent-descent.
  Releasing sticks levels the vehicle, actively brakes it to a stop, and locks it to the current 3D position (even against wind and other forces).
- [Position Slow mode](../flight_modes_mc/position_slow.md) — A velocity and yaw rate limited version of _Position mode_.
  This is primarily used to temporarily limit speed when flying near obstacles, or when required by regulation.
- [Altitude mode](../flight_modes_mc/altitude.md) — Easiest and safest _non-GPS_ manual mode.
  The main difference when compared to _Position mode_ is that when the sticks are released the vehicle will level and maintain altitude, but there is no active breaking or holding of horizontal position (the vehicle moves with it's current momentum and drifts with wind).
- [Stabilized mode](../flight_modes_mc/manual_stabilized.md) — Releasing the sticks levels and maintains the vehicle horizontal posture (but not altitude or position).
  The vehicle will continue to move with momentum, and both altitude and horizontal position may be affected by wind.
  This mode is also used if "Manual mode" is selected in a ground station.

Manual-Acrobatic

- [Acro](../flight_modes_mc/acro.md) — Manual mode for performing acrobatic maneuvers, such as rolls and loops.
  Releasing the sticks stops the vehicle rotating in the roll, pitch, yaw axes, but does not otherwise stabilise the vehicle.

Autonomous:

- [Hold](../flight_modes_mc/hold.md) — Vehicle stops and hovers at its current position and altitude, maintaining its position against wind and other forces.
- [Return](../flight_modes_mc/return.md) — Vehicle ascends to a safe altitude, flies a clear path to a safe location (home or a rally point) and then lands.
  This requires a global position estimate (GPS).
- [Mission](../flight_modes_mc/mission.md) — Vehicle executes a [predefined mission/flight plan](../flying/missions.md) that has been uploaded to the flight controller.
  This requires a global position estimate (GPS).
- [Takeoff](../flight_modes_mc/takeoff.md) — Vehicle takes off vertically and then switches to _Hold mode_.
- [Land](../flight_modes_mc/land.md) — Vehicle lands immediately.
- [Orbit](../flight_modes_mc/orbit.md) - Vehicle flys in a circle, yawing so that it always faces towards the center.
  RC control can optionally be used to change the orbit radius, direction, speed and so on.
- [Follow Me](../flight_modes_mc/follow_me.md) — Vehicle follows a beacon that is providing position setpoints.
  RC control can optionally be used to set the follow position.
- [Offboard](../flight_modes_mc/offboard.md) — Vehicle obeys position, velocity, or attitude, setpoints provided via MAVLink or ROS 2.

Pilots transition between flight modes using switches on the remote control or with a ground control station (see [Flight Mode Configuration](../config/flight_mode.md)).
Some flight modes make sense only under specific pre-flight and in-flight conditions (e.g. GPS lock, airspeed sensor, vehicle attitude sensing along an axis).
PX4 will not allow transitions to those modes until the right conditions are met.

Select the mode-specific sidebar topics for more detailed technical information.

## Further Information

- [Basic Configuration > Flight Modes](../config/flight_mode.md) - How to map RC control switches to specific flight modes
- [Flight Modes (Fixed-Wing)](../flight_modes_fw/index.md)
- [Flight Modes (VTOL)](../flight_modes_vtol/index.md)
- [Drive Modes (Differential Rover)](../flight_modes_rover/differential.md)
- [Drive Modes (Ackermann Rover)](../flight_modes_rover/ackermann.md)
