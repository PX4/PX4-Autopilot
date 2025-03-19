# Flight Modes (Fixed-Wing)

Flight modes provide autopilot support to make it easier to manually fly the vehicle, to automate common tasks such as takeoff and landing, to execute autonomous missions, or to defer flight control to an external system.

This topic provides an overview of the available flight modes for fixed-wing vehicles (planes).

## 개요

Flight Modes are either _manual_ or _autonomous_.
Manual modes provide different levels of autopilot support when flying manually (using RC control sticks or a joystick), while _autonomous_ modes are fully controlled by the autopilot.

Manual-Easy:

- [Position mode](../flight_modes_fw/position.md) — Easiest and safest manual mode for vehicles that have a position fix/GPS.
  The vehicle performs a [coordinated turn](https://en.wikipedia.org/wiki/Coordinated_flight) if the roll stick is non-zero, while the pitch stick controls the rate of ascent/descent.
  If the sticks are released the vehicle levels out and holds a straight flight path, even against wind.
  Airspeed is actively controlled if an airspeed sensor is installed.
- [Altitude](../flight_modes_fw/altitude.md) — Easiest and safest _non-GPS_ manual mode.
  The only difference compared to _Position mode_ is that the pilot always directly controls the roll angle of the plane and there is no automatic course holding.
- [Stabilized mode](../flight_modes_fw/stabilized.md) — The pilot directly commands the roll and pitch angle and the vehicle keeps the setpoint until the sticks are moved again.
  Thrust is directly set by the pilot.
  Turn coordination is still handled by the controller.
  Height and airspeed are not controlled, in particular it is the pilot's responsibility to not stall the vehicle.

Manual-Acrobatic

- [Acro mode](../flight_modes_fw/acro.md) — Manual mode for performing acrobatic maneuvers, such as rolls and flips, stalls and acrobatic figures.
  The roll, pitch, and yaw, sticks control the rate of angular rotation around the respective axes and throttle is passed directly to control allocation. When sticks are centered the vehicle will stop rotating, but remain in its current orientation (on its side, inverted, or whatever) and moving according to its current momentum.
- [Manual](../flight_modes_fw/manual.md) — Hardest manual flight mode.
  This sends stick input directly to control allocation for "fully" manual control.
  No sensor feedback is used to compensate for disturbances.

Autonomous:
All autonomous flight modes require a valid position estimate (GPS).
Airspeed is actively controlled if an airspeed sensor is installed in any autonomous flight mode.

- [Hold](../flight_modes_fw/hold.md) — Vehicle circles around the GPS hold position at the current altitude.
  The mode can be used to pause a mission or to help regain control of a vehicle in an emergency.
  It can be activated with a pre-programmed RC switch or the QGroundControl Pause button.
- [Return](../flight_modes_fw/return.md) — Vehicle flies a clear path to land at a safe location.
  By default the destination is a mission landing pattern.
  The mode may be activated manually (via a pre-programmed RC switch) or automatically (i.e. in the event of a failsafe being triggered).
- [Mission](../flight_modes_fw/mission.md) — Vehicle executes a [predefined mission/flight plan](../flying/missions.md) that has been uploaded to the flight controller.
- [Takeoff](../flight_modes_fw/takeoff.md) — Vehicle initiates the takeoff sequence using either _catapult/hand-launch mode_ or _runway takeoff mode_ (in the current direction).
- [Land](../flight_modes_fw/land.md) — Vehicle initiates the [fixed-wing landing sequence](../flight_modes_fw/mission.md#mission-landing).
- [Offboard](../flight_modes_fw/offboard.md) — Vehicle obeys attitude setpoints provided via MAVLink or ROS 2.

Pilots transition between flight modes using switches on the remote control or with a ground control station (see [Flight Mode Configuration](../config/flight_mode.md)).
Some flight modes make sense only under specific pre-flight and in-flight conditions (e.g. GPS lock, airspeed sensor, vehicle attitude sensing along an axis).
PX4 will not allow transitions to those modes until the right conditions are met.

Select the mode-specific sidebar topics for detailed technical information.

## 추가 정보

- [Basic Configuration > Flight Modes](../config/flight_mode.md) - How to map RC control switches to specific flight modes
- [Flight Modes (Multicopter)](../flight_modes_mc/index.md)
- [Flight Modes (VTOL)](../flight_modes_vtol/index.md)
- [Drive Modes (Differential Rover)](../flight_modes_rover/differential.md)
- [Drive Modes (Ackermann Rover)](../flight_modes_rover/ackermann.md)