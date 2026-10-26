# Navigation Terminology (Advanced)

This page is the developer reference behind [Navigation Terminology](../concept/navigation_terminology.md).
It is aimed at contributors who add or change uORB fields, parameters, MAVLink handling, log analysis tools or documentation.

PX4 flies aircraft, drives rovers, and can drive boats, so its code and documentation use the vocabulary of aviation, marine navigation and ground vehicle dynamics.
These traditions use the same words (_heading_, _course_, _track_, _bearing_, _drift_, _sideslip_), and they do not always mean the same thing.

This page collects the official definitions, the reference frames they are measured in, and the rule PX4 applies when they conflict.
PX4 uses these definitions as they are: it does not define its own navigation vocabulary.
Where PX4 represents or computes something differently from the norms, this is described separately in [Where PX4 Differs](#where-px4-differs).

## Sources and Precedence

The definitions are taken from these authorities (see [References](#references)):

- **Aviation:** ICAO (Annex 2, Doc 9713), FAA _Pilot's Handbook of Aeronautical Knowledge_ (PHAK) and FAA _Pilot/Controller Glossary_ (P/CG).
- **Marine:** NGA _The American Practical Navigator_ (Bowditch, Pub. No. 9), IHO S-32 _Hydrographic Dictionary_, NMEA 0183.
- **Ground vehicles:** ISO 8855 (road vehicle dynamics vocabulary) and SAE J670 (vehicle dynamics terminology).
- **Frames:** WGS 84 (NGA, adopted by ICAO), ISO 1151 (flight dynamics), SNAME (marine vehicle motion) and ISO 8855 (road vehicles).

Where definitions of the same word conflict between these traditions, PX4 applies this rule:

1. **Aviation definitions take precedence** (ICAO first, then FAA).
   They are international standards, and most existing PX4 interfaces already follow them.
2. **Marine terms are used for marine concepts** that aviation does not define, such as speed through the water, and leeway and set and drift (see [Terms Not Yet Used in PX4](#terms-not-yet-used-in-px4)).
3. **Ground vehicle terms are used for vehicle dynamics concepts** that navigation does not define, such as tyre slip angle (see [Terms Not Yet Used in PX4](#terms-not-yet-used-in-px4)).
   For directions (heading, course, drift angle), the navigation terms apply to rovers too.
4. **Where a word is ambiguous between traditions, use the official qualified term**: for example _desired track_ (FAA P/CG) instead of bare _track_ for an intended path, and _drift angle_ (FAA PHAK) instead of bare _drift_.

The conflicts and the resulting choices are listed in [Terminology Conflicts](#terminology-conflicts).

## Frames and Reference Directions

Every position and direction is relative to a reference frame.
This section describes the frames PX4 uses, and the official convention each one is based on.

### Geodetic Frame (WGS 84)

Global positions are given as latitude, longitude and altitude on the World Geodetic System 1984 (WGS 84), the reference system that ICAO requires for aeronautical data and that GNSS receivers use.

- **Latitude** and **longitude** are in degrees, positive north and east.
- **Altitude** has two different references, which must never be mixed up:
  - Height above the WGS 84 **ellipsoid**, as output by GNSS receivers.
  - Altitude above mean sea level (**AMSL**), which differs from the ellipsoid height by the local geoid height (up to about 100 m).

In PX4, `vehicle_global_position` carries `lat`, `lon`, `alt` (AMSL) and `alt_ellipsoid`.
On MAVLink and in mission items, the reference of each altitude is given by its `MAV_FRAME`.

### Local Earth-Fixed Frame (NED)

ISO 1151 defines the _normal earth-fixed axis system_: an earth-fixed frame whose z-axis points vertically down.
Aviation and marine engineering conventionally align its horizontal axes with north and east, which gives the **north-east-down (NED)** frame:

- x points to true north, y points to east, z points down.
- It is a plane tangent to the Earth at a chosen origin.
- It is right-handed, so positive rotation about z is clockwise when viewed from above.
  This matches the clockwise sense of headings, courses and bearings.

In PX4, the local frame of `vehicle_local_position` (fields `x`, `y`, `z`, `vx`, `vy`, `vz`) is NED.
Its origin is the position where the estimator was initialised (`ref_lat`, `ref_lon`, `ref_alt`).
This is the same frame as MAVLink `MAV_FRAME_LOCAL_NED`.

See [Local Frame Is a Map Projection](#local-frame-is-a-map-projection) for how PX4 relates the local frame to the geodetic frame.

### Body Frame (FRD)

ISO 1151 (aircraft) and SNAME (ships and submarines) define the body-fixed axes the same way:

- x points forward (nose, bow or front).
- y points to the right.
- z points down, completing a right-handed frame.

PX4 calls this frame **FRD** (front, right, down), which is the same frame as MAVLink `MAV_FRAME_BODY_FRD`.
The vehicle's attitude is the rotation from NED to FRD, and its [heading](#heading) is the direction of the body x-axis in the horizontal plane.

::: info
Some ground vehicle and robotics conventions use z-up frames instead:

- ISO 8855 defines the road vehicle axis system with x forward, y to the left and z up (FLU), and an earth-fixed frame with z up.
  Angles about z are then positive counterclockwise when viewed from above, the opposite of navigation practice.
  SAE J670 allows both the ISO z-up and the traditional SAE z-down axis systems, so check which one a source uses.
- ROS uses ENU for the local frame and FLU for the body frame.
  See [ROS 2 & PX4 Frame Conventions](../ros2/user_guide.md#ros-2-px4-frame-conventions).

To convert from FLU to FRD, negate y and z, and negate all angles and rates about z.
:::

### Track Frame (Along-Track and Cross-Track)

To describe how well a vehicle follows a path, area navigation (RNAV, ICAO Doc 9613) splits the horizontal position error into two components relative to the [desired track](#track):

- **Along-track:** along the desired track, in the direction of travel.
- **Cross-track:** perpendicular to the desired track (see [Cross-Track Error](#cross-track-error)).

PX4 path-following controllers (NPFG, pure pursuit) work in this frame.

_Desired track from waypoint A to waypoint B, with the along-track axis along the line, the cross-track axis to the right, and the vehicle position decomposed into both._

### North References

ICAO defines directions as measured "from North (true, magnetic, compass or grid)", and Bowditch uses the same four references:

- **True north:** The direction of the geographic North Pole along the local meridian.
- **Magnetic north:** The direction of the horizontal component of the Earth's magnetic field.
  The angle between true and magnetic north is called _magnetic variation_ in aviation (FAA PHAK) and _variation_ in marine navigation (Bowditch).
  Geomagnetic models such as the World Magnetic Model call the same angle _magnetic declination_.
- **Compass north:** The direction an installed compass indicates.
  It differs from magnetic north by the _deviation_ caused by the vehicle's own magnetic fields.
- **Grid north:** The direction of the north axis of a map projection grid.
  It differs from true north away from the projection's central meridian.

PX4 uses **true north** internally.
The magnetometer measures compass north, so the estimator applies the calibration (which removes the deviation) and the declination (from the World Magnetic Model, or from [EKF2_MAG_DECL](../advanced_config/parameter_reference.md#EKF2_MAG_DECL) depending on [EKF2_DECL_TYPE](../advanced_config/parameter_reference.md#EKF2_DECL_TYPE)).

### Angle Conventions

The aviation and marine norms express directions in degrees, measured **clockwise from north from 000° to 360°**.
East is 090°, south is 180°, west is 270°.
MAVLink follows this convention for navigation angles (for example `GPS_RAW_INT.cog` in centidegrees from 0 to 35999).

PX4 uses the same zero and the same sense of rotation, but a different unit and range internally: see [Angles in Radians](#angles-in-radians).

## Definitions by Tradition

The [Navigation Terminology](../concept/navigation_terminology.md) page gives one definition per term.
This section lists the definitions, where they differ, and the choice PX4 makes.

### Heading

> **ICAO:** "The direction in which the longitudinal axis of an aircraft is pointed, usually expressed in degrees from North (true, magnetic, compass or grid)."
> — ICAO Doc 9713, _International Civil Aviation Vocabulary_, and Annex 2, _Rules of the Air_

> **FAA:** "Heading—direction in which the nose of the aircraft points during flight."
> — FAA PHAK, Chapter 16

> **Bowditch:** "Heading (Hdg., SH) is the direction in which a vessel is pointed at any given moment, expressed as angular distance from 000° clockwise through 360°. It is easy to confuse heading and course."
> — Bowditch (2019), Art. 107

Aviation and marine definitions agree, and ground vehicle dynamics (ISO 8855, SAE J670) calls the same angle the _yaw angle_ (see [Ground Vehicles](#ground-vehicles-yaw-course-and-sideslip-angle)).

### Course

> **FAA:** "Course—intended path of an aircraft over the ground or the direction of a line drawn on a chart representing the intended aircraft path, expressed as the angle measured from a specific reference datum clockwise from 0° through 360° to the line."
> — FAA PHAK, Chapter 16

> **FAA:** "COURSE - The intended direction of flight in the horizontal plane measured in degrees from north."
> — FAA P/CG

> **Bowditch:** "Course (C, Cn) is the horizontal direction in which a vessel is intended to be steered, expressed as angular distance from north clockwise through 360°. Strictly used, the term applies to direction through the water, not the direction intended to be made good over the ground."
> — Bowditch (2019), Art. 107

The course is an _intended_ direction.
In aviation it is intended over the ground, while in strict marine usage it is the direction steered through the water.
Following the [precedence rule](#sources-and-precedence), PX4 uses the aviation meaning: the intended direction of motion over the ground.

In PX4: `fixed_wing_lateral_setpoint.course` and `position_setpoint.course`.

### Course Over Ground

> **IHO:** "course over ground: The direction of the path over the ground actually followed by a vessel. It is normally a somewhat irregular line."
> — IHO S-32

Course over ground (COG) is the _actual_ direction of motion over the ground: the direction of the horizontal ground velocity vector.
With $v_N$ and $v_E$ the north and east components of the ground velocity:

$$
\chi = \operatorname{atan2}(v_E, v_N)
$$

Ground vehicle dynamics (ISO 8855, SAE J670) calls the same angle the _course angle_ (see [Ground Vehicles](#ground-vehicles-yaw-course-and-sideslip-angle)).

It is the direction of the [track](#track) at the current point.
This is why avionics display it as _TRK_ (track angle), and why NMEA 0183 `VTG` labels it "track made good".
COG is undefined when the vehicle is stationary and noisy at low speed, so code that uses it must check a minimum ground speed or the course accuracy.

In PX4: `sensor_gnss.course`, MAVLink `GPS_RAW_INT.cog`, NMEA `RMC` and `VTG`.

### Track

> **ICAO:** "TRACK [ICAO] - The projection on the earth's surface of the path of an aircraft, the direction of which path at any point is usually expressed in degrees from North (True, Magnetic, or Grid)."
> — FAA P/CG, quoting ICAO

> **FAA:** "Track—actual path made over the ground in flight. (If proper correction has been made for the wind, track and course are identical.)"
> — FAA PHAK, Chapter 16

> **FAA:** "DESIRED TRACK - The planned or intended track between two waypoints. It is measured in degrees from either magnetic or true north. The instantaneous angle may change from point to point along the great circle track between waypoints."
> — FAA P/CG

> **Bowditch:** "Track (TR) is the intended horizontal direction of travel with respect to the Earth. The terms intended track and trackline are used to indicate the path of intended travel."
> — Bowditch (2019), Art. 107

In aviation the **track** is the _actual_ path over the ground, and the **desired track** is the _intended_ path between two waypoints.
In marine navigation the track is the _intended_ path.

Following the [precedence rule](#sources-and-precedence), PX4 uses the aviation terms:

- **Track:** The actual path over the ground.
  Its direction at any point is the [course over ground](#course-over-ground).
- **Desired track:** The intended path over the ground, from which the [cross-track error](#cross-track-error) is measured.
  This is usually the line between two waypoints, but it can also be a line along a course the vehicle holds, which the FAA definition does not cover.

Marine readers understand bare "track" as the intended path, so always write _desired track_ for the intended path.

### Bearing

> **FAA:** "BEARING - The horizontal direction to or from any point, usually measured clockwise from true north, magnetic north, or some other reference point through 360 degrees."
> — FAA P/CG

> **Bowditch:** "Bearing (B, Brg.) is the direction of one terrestrial point from another, expressed as angular distance from 000° (North) clockwise through 360°."
> — Bowditch (2019), Art. 107

Aviation and marine definitions agree.

::: warning
A bearing is not a commanded direction.
The direction a guidance law commands is a course (or heading) setpoint.
See [Field Names That Deviate](#field-names-that-deviate).
:::

### Cross-Track Error

Cross-track error (XTE) is a standard output of RNAV systems and marine navigation receivers.
Bowditch lists it alongside course over ground, speed over ground and bearing to the next waypoint (Art. 2501).
NMEA 0183 reports it in the `XTE`, `APB` and `RMB` sentences, as a distance together with the direction to steer (`L` or `R`) to return to the desired track.

The cross-track error is the perpendicular distance of the vehicle from the [desired track](#track), together with the side of the track the vehicle is on.
The norms do not define a sign: see [Signed Cross-Track Error](#signed-cross-track-error) for how PX4 represents it.

### Drift Angle

> **FAA:** "Drift angle—angle between heading and track."
> — FAA PHAK, Chapter 16

The drift angle is caused by wind for aircraft, and by wind and current for vessels.
For ground vehicles, it is caused by wheel slip, and ISO 8855 calls it the _vehicle sideslip angle_ (see [Ground Vehicles](#ground-vehicles-yaw-course-and-sideslip-angle)).

::: warning
Marine _drift_ is the **speed** of a current, while the aviation _drift angle_ is an **angle** between heading and track.
Always write _drift angle_ for the angle, and never bare _drift_.
:::

::: info
For aircraft, the drift angle is not the _sideslip angle_ ($\beta$, see [Terminology/Notation](../contribute/notation.md)).
In flight dynamics (ISO 1151), sideslip is measured against the airflow, so a plane that crabs cleanly through a crosswind has a large drift angle and zero sideslip.
Ground vehicle dynamics uses the same name for a ground-referenced angle: see [Ground Vehicles](#ground-vehicles-yaw-course-and-sideslip-angle).
:::

In PX4, the aerodynamic sideslip appears for example in `estimator_innovations.beta` (EKF2 synthetic sideslip fusion).

### Ground Vehicles: Yaw, Course and Sideslip Angle

ISO 8855 and SAE J670 describe the horizontal motion of a ground vehicle with three angles, measured in the ground plane:

- **Yaw angle** ($\psi$): The angle from the earth-fixed x-axis to the vehicle x-axis.
  This is the [heading](#heading).
- **Course angle** ($\nu$): The angle from the earth-fixed x-axis to the horizontal velocity of the vehicle over the ground.
  This is the actual direction of motion, which is the [course over ground](#course-over-ground), **not** the intended [course](#course) of the FAA definition.
- **Sideslip angle** ($\beta = \nu - \psi$): The angle from the vehicle x-axis to the horizontal velocity over the ground.
  This is the ground vehicle counterpart of the [drift angle](#drift-angle), and **not** the air-relative sideslip angle of flight dynamics.

ISO 8855 measures these angles counterclockwise about a z-up axis, so its sideslip angle has the opposite sign of the [PX4 drift angle](#drift-angle-sign).

Following the [precedence rule](#sources-and-precedence), use these terms for rovers:

- _Heading_ for the yaw angle, _course over ground_ for the course angle and _drift angle_ for the vehicle sideslip angle.
- When code or documentation follows vehicle dynamics literature, the ISO 8855 terms may be used, but write _vehicle sideslip angle_ and cite ISO 8855, so that it is not confused with aerodynamic sideslip.

For a rover that drives without wheel slip (Ackermann or differential drive), the course over ground equals the heading when driving forwards, and the heading plus 180° when reversing.
A drift angle appears when the wheels slip, for example on loose ground or in fast turns, and by design on mecanum rovers.

### Airspeed, Ground Speed and Speed Over Ground

> **FAA:** "Airspeed—rate of the aircraft's progress through the air." "GS—rate of the aircraft's inflight progress over the ground."
> — FAA PHAK, Chapter 16

> **Bowditch:** "Speed over the ground (SOG) is the actual speed of the vessel over the surface of the Earth at any given time."
> — Bowditch (2019), Art. 106

Ground speed (aviation, FAA PHAK) and speed over ground (marine) are the same quantity: the magnitude of the horizontal ground velocity, whose direction is the course over ground.
For vessels, the water-relative counterpart of airspeed is the _speed through the water_, which a log (paddle wheel) measures and a GNSS receiver does not.

### Wind and Current Triangle

The [wind triangle](../concept/navigation_terminology.md#wind-triangle) has a marine counterpart for vessels:

$$
\mathbf{v}_{ground} = \mathbf{v}_{water} + \mathbf{v}_{current}
$$

The current is described by its [set and drift](#set-and-drift), and $\mathbf{v}_{water}$ is offset from the heading by [leeway](#leeway).

## Terminology Conflicts

These are the terms where the traditions disagree, and the official term PX4 uses as a result of the [precedence rule](#sources-and-precedence).
Heading (yaw angle in ISO 8855), bearing, course over ground and cross-track error mean the same in all of them.

| Term                   | Aviation (ICAO/FAA, ISO 1151)                                     | Marine (Bowditch)                                  | Ground vehicles (ISO 8855/SAE J670)                               | Term to use in PX4                                                                              |
| ---------------------- | ----------------------------------------------------------------- | -------------------------------------------------- | ----------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| Track                  | Actual path over the ground; the intended path is _desired track_ | Intended path over the ground                      | —                                                                 | _Track_ (actual) and _desired track_ (intended)                                                 |
| Course                 | Intended direction over the ground                                | Direction steered through the water (strict sense) | _Course angle_: actual direction of motion over the ground        | _Course_ (intended) and _course over ground_ (actual)                                           |
| Drift                  | _Drift angle_: angle between heading and track                    | Speed of the current, with _set_ as its direction  | —                                                                 | _Drift angle_ (angle); never bare _drift_                                                       |
| Sideslip               | Angle between the body x-axis and the airflow                     | —                                                  | Angle between the vehicle x-axis and the velocity over the ground | _Sideslip angle_ (air-relative) and _drift angle_ or _vehicle sideslip angle_ (ground-relative) |
| True to magnetic north | _Magnetic variation_                                              | _Variation_                                        | —                                                                 | _Declination_ (World Magnetic Model term, used by EKF2)                                         |

## Where PX4 Differs

PX4 follows the official definitions above.
The sections below describe where PX4 represents, computes or names a quantity differently from the norms, so that you can convert correctly.

### Angles in Radians

The norms express directions in degrees from 000° to 360°.
In uORB, PX4 expresses them in **radians, wrapped to $[-\pi, \pi]$**, with the same zero (true north) and the same sense (clockwise).
East is $+\pi/2$ and west is $-\pi/2$.

Convert with `matrix::wrap_2pi()` and `math::degrees()` when you output to MAVLink or to users, as the MAVLink streams do.

### Local Frame Is a Map Projection

The estimator keeps the vehicle's global position (latitude and longitude), and derives the local position from it with an **azimuthal equidistant projection** centred on the local origin (`MapProjection` in `src/lib/geo`).
As a result:

- The x-axis of `vehicle_local_position` points to true north **at the origin**.
  At the vehicle's position it points to the grid north of the projection, which differs from true north by the meridian convergence.
  The difference grows with distance from the origin, mainly east or west of it.
  It is about 0.1° at 10 km east of an origin at 47° latitude.
- Heading and velocity are relative to true north **at the vehicle**, not to the projection's grid north.
- A straight line between two waypoints in the local frame is a straight line in the projection.
  The FAA desired track follows the great circle between the waypoints.
  The two are identical for lines through the origin, and practically identical over typical mission distances.

For vehicles that operate tens of kilometres from the origin, take this into account when you compare local-frame directions with true directions.

### Signed Cross-Track Error

The norms report cross-track error as a distance and a side (NMEA: the direction to steer).
PX4 represents it as a single **signed** distance:

- It is the shortest horizontal distance from the vehicle to the desired track.
- It is **positive when the vehicle is to the right of the desired track** and negative when it is to the left, looking in the direction of travel.

With $\mathbf{t}$ the unit vector along the desired track and $\mathbf{d}$ the vector from a point on the track to the vehicle (north and east components):

$$
e_{xt} = t_N d_E - t_E d_N
$$

This is the convention used by NPFG (`fixed_wing_lateral_guidance_status.signed_track_error`) and by pure pursuit (`pure_pursuit_status.crosstrack_error`).
A vehicle with a positive cross-track error gets NMEA "steer L".

### Drift Angle Sign

The FAA defines the drift angle as the angle between heading and track, without a sign.
Where PX4 needs a signed value, it is `wrap_pi(course_over_ground - heading)`: positive when the vehicle moves to the right of where its nose points.

### Wind as a Velocity Vector

The norms report wind by the direction it blows _from_.
PX4 wind estimates (`wind`, `airspeed_wind`) are velocity vectors in NED (`windspeed_north`, `windspeed_east`), which point in the direction the air moves _toward_.
A wind reported as "from 360°" has `windspeed_north < 0` in PX4.

### Field Names That Deviate

Some existing names predate these pages or come from external protocols, and do not match the official terms.
Read them with the meaning given here, and use the official term in new code:

| Field                                                                                                                                                                                                | Actual meaning                                                                                                                        | Official term              |
| ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- | -------------------------- |
| `position_setpoint.course` (comment: "desired course (bearing)")                                                                                                                                     | Intended direction of motion over the ground.                                                                                         | Course                     |
| `position_controller_status.nav_bearing` (MAVLink `NAV_CONTROLLER_OUTPUT.nav_bearing`)                                                                                                               | Direction commanded by the guidance law.                                                                                              | Course or heading setpoint |
| `position_controller_status.xtrack_error`                                                                                                                                                            | Signed cross-track error.                                                                                                             | Cross-track error          |
| `fixed_wing_lateral_guidance_status.bearing_feas`                                                                                                                                                    | Feasibility of the course commanded by NPFG (NPFG calls it "bearing").                                                                | Course                     |
| `transponder_report.heading` (MAVLink `ADSB_VEHICLE.heading`)                                                                                                                                        | Course over ground of the other aircraft, as reported by ADS-B.                                                                       | Course over ground         |
| `bearing_setpoint` in the rover position controllers (for example `AckermannPosControl`)                                                                                                             | Course setpoint, taken from the bearing to the pure pursuit lookahead point (`PurePursuit::calcTargetBearing()`, which is a bearing). | Course setpoint            |
| "Direction (bearing)" of the velocity setpoint in [Rover Velocity Tuning](../config_rover/velocity_tuning.md) and "bearing setpoints" in [Rover Position Tuning](../config_rover/position_tuning.md) | Intended direction of motion over the ground.                                                                                         | Course                     |

## Guidelines for Contributors

When you add or change uORB fields, parameters, MAVLink handling, log analysis tools or documentation:

- **Use the official terms on these pages. Do not introduce new navigation terms.**
  If a concept is missing, look up its official name in the [references](#references) and add it here.
- Use _heading_ only for the direction the vehicle points, _course_ only for the intended direction over the ground, _course over ground_ for the actual direction over the ground, and _bearing_ only for the direction from one point to another.
- Write _desired track_ for an intended path, and never use bare _drift_.
- Use _sideslip angle_ only for the angle against the airflow. For the ground-relative angle, use _drift angle_, or _vehicle sideslip angle_ with a reference to ISO 8855.
- Use the PX4 [signed cross-track error](#signed-cross-track-error) convention.
- State the reference (true north), unit and range in the field comment, and add the `[@frame]` and `[@range]` tags (see [uORB Documentation Standard](../uorb/uorb_documentation.md)).
- When you convert to or from an external protocol, check that protocol's definition of each field: names alone are not reliable.

## Terms Not Yet Used in PX4

These terms are defined by the norms, but PX4 does not use them yet.
They are kept here for reference, so that new code uses the official term when it needs one of these concepts.

:::: details Show terms not yet used in PX4

| Term                                            | Official meaning                                                                         | Source             |
| ----------------------------------------------- | ---------------------------------------------------------------------------------------- | ------------------ |
| [Track made good](#track-made-good)             | Net direction from the point of departure to the point of arrival.                       | Bowditch           |
| [Relative bearing](#relative-bearing)           | Bearing measured from the vehicle's heading.                                             | Bowditch           |
| [Wind correction angle](#wind-correction-angle) | Correction applied to the course to obtain a heading that keeps the track on the course. | FAA                |
| [Tyre slip angle](#tyre-slip-angle)             | Angle between the wheel plane and the direction of travel of the tyre contact patch.     | ISO 8855, SAE J670 |
| [Leeway](#leeway)                               | Leeward motion of a vessel due to the wind.                                              | Bowditch           |
| [Set and drift](#set-and-drift)                 | Direction (toward) and speed of a current.                                               | Bowditch           |

#### Track Made Good

> **Bowditch:** "Track made good (TMG) is the single resultant direction from the point of departure to point of arrival at any given time. The use of this term is preferred to the use of the misnomer 'course made good.'"
> — Bowditch (2019), Art. 107

Track made good (also called course made good, CMG) is the net direction between two positions, not an instantaneous value.
The matching speed is the _speed made good_ (SMG): the distance between the two positions divided by the elapsed time.
PX4 controllers do not use it, but it is useful in log analysis.

#### Relative Bearing

> **Bowditch:** "A relative bearing is measured relative to the ship's heading from 000° (dead ahead) clockwise through 360°."
> — Bowditch (2019), Art. 107

- **Relative bearing:** The [bearing](#bearing) measured from the vehicle's heading.
  Bowditch: "True Bearing = Relative Bearing + True Heading."
- **Reciprocal bearing:** The bearing in the opposite direction, which differs by 180°.

#### Wind Correction Angle

> **FAA:** "WCA—correction applied to the course to establish a heading so that track coincides with course."
> — FAA PHAK, Chapter 16

The **wind correction angle** (WCA, also called _crab angle_) is the angle by which the heading is offset from the course into the wind, so that the track stays on the course.
In the [wind triangle](../concept/navigation_terminology.md#wind-triangle), it is the angle between the heading and the course.

#### Tyre Slip Angle

ISO 8855 and SAE J670 define the _slip angle_ of each tyre: the angle between the wheel plane and the direction of travel of the wheel's contact patch.
It describes how a tyre generates lateral force, and is a vehicle dynamics term with no navigation counterpart.
Write _tyre slip angle_ for it, to distinguish it from the [vehicle sideslip angle](#ground-vehicles-yaw-course-and-sideslip-angle).

#### Leeway

> **Bowditch:** "Leeway is the leeward motion of a vessel due to that component of the wind vector perpendicular to the vessel's track."
> — Bowditch (2019), Art. 1006

Leeway is the sideways push of the wind on a vessel's hull and superstructure, relative to the water.
It is the marine counterpart of an aircraft being displaced by the wind, and contributes to the drift angle.
In the [current triangle](#wind-and-current-triangle), $\mathbf{v}_{water}$ is offset from the heading by leeway.

#### Set and Drift

> **Bowditch:** "Set refers to the current's direction, and drift refers to the current's speed."
> — Bowditch (2019), Art. 1006

The _set_ is the direction **toward** which the current flows, and the _drift_ is its speed.
When a dead reckoning (DR) position is compared with a fix, the same terms describe the total unexplained offset: "The set is the direction from the DR to the fix" (Bowditch, Art. 1008).

This is the opposite of [wind direction](../concept/navigation_terminology.md#wind-direction), which is the direction the wind blows **from**.
A current with set 000° and a wind from 180° push a vehicle the same way.
PX4 wind estimates are velocity vectors, so they [point the same way as current set](#wind-as-a-velocity-vector).

::: warning
Marine _drift_ is a **speed** of the current, while aviation _drift angle_ is an **angle** between heading and track.
Write _drift angle_ for the angle, and _set and drift_ (of the current) for the current.
:::

::::

## References

Aviation:

- ICAO Doc 9713, _International Civil Aviation Vocabulary_, and ICAO Annex 2, _Rules of the Air_ (heading, track).
- ICAO Doc 9613, _Performance-based Navigation (PBN) Manual_ (along-track and cross-track components).
- ICAO Doc 9674, _World Geodetic System — 1984 (WGS-84) Manual_.
- FAA, [Pilot's Handbook of Aeronautical Knowledge](https://www.faa.gov/regulations_policies/handbooks_manuals/aviation/phak) (FAA-H-8083-25), Chapter 16: Navigation.
- FAA, [Pilot/Controller Glossary](https://www.faa.gov/air_traffic/publications/atpubs/pcg_html/) (BEARING, COURSE, DESIRED TRACK, TRACK, TRACK [ICAO]).
- SKYbrary, [Heading, Track and Radial](https://skybrary.aero/articles/heading-track-and-radial) (summary of the ICAO definitions).

Marine:

- National Geospatial-Intelligence Agency, [The American Practical Navigator (Bowditch), Pub. No. 9](https://msi.nga.mil/Publications/APN), 2019 edition, Volume I: Art. 106 and 107 (speed and direction), Art. 1006 and 1008 (leeway, set and drift), Art. 2501 (receiver outputs, including cross-track error).
- International Hydrographic Organization, [S-32 Hydrographic Dictionary](https://portal.iho.int/iho-ohi/S32/engView.php?page=56) (course over ground, course made good).
- National Marine Electronics Association, [NMEA 0183](https://www.nmea.org/nmea-0183.html) (`RMC`, `VTG`, `HDT`, `XTE`, `APB`, `RMB`).
- SNAME, _Nomenclature for Treating the Motion of a Submerged Body Through a Fluid_ (body axes for marine vehicles).

Ground vehicles:

- ISO 8855:2011, [Road vehicles — Vehicle dynamics and road-holding ability — Vocabulary](https://www.iso.org/standard/51180.html) (axis systems, yaw, course and sideslip angle, tyre slip angle).
- SAE J670 (2022), [Vehicle Dynamics Terminology](https://saemobilus.sae.org/standards/j670_202206-vehicle-dynamics-terminology).

Frames:

- ISO 1151, _Flight dynamics — Concepts, quantities and symbols_ (earth-fixed and body axis systems, aerodynamic sideslip).
- National Geospatial-Intelligence Agency, _World Geodetic System 1984_ (NGA.STND.0036).

PX4:

- [Navigation Terminology](../concept/navigation_terminology.md): the commonly used terms.
- [Terminology/Notation](../contribute/notation.md): symbols, frames and acronyms.
- [uORB Documentation Standard](../uorb/uorb_documentation.md): how to document frames, units and ranges of message fields.
