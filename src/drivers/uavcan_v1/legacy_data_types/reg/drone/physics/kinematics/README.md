# Kinematic states

This namespace contains data types that model basic [kinematic](https://en.wikipedia.org/wiki/Kinematics) states.

The full kinematic state of a rigid body or fluid includes its position, velocity, acceleration, and orientation.
The data types contained here model either full or partial kinematic states (e.g., there are types for velocity only).

Forces acting on the body or fluid are part of its *dynamic* state, so they are excluded from the model.

The modeled kinematic state of a body in `cartesian` space includes its pose (position and orientation)
and twist (here, twist is understood narrowly as the translational and rotational velocity of the body).
A "point state" lacks the rotational component (only position and velocity are defined).
Translational components are always specified before their rotational counterparts.

There is a dedicated namespace `geodetic` that defines position in spherical coordinates instead of Cartesian.
This is to support large-scale navigation along the surface of celestial bodies where a local
Cartesian approximation is infeasible.
Other than using a different coordinate system to express position, the data types are equal to their Cartesian
counterparts and are to some extent interchangeable thanks to the structural subtyping/aliasing features of DSDL.

See UAVCAN Specification chapter "Application layer" for the applicable conventions.
Key excerpts:

- For world fixed frames, the North-East-Down (NED) right-handed notation is preferred:
  X – northward, Y – eastward, Z – downward.

- In relation to a body, the convention is as follows, right-handed:
  X – forward, Y – rightward, Z – downward.

- Angular velocities are represented using the right-handed, fixed-axis (extrinsic) convention:
  X (roll), Y (pitch), Z (yaw).

- For NED frames, the initial (zero) rotation of a body is the state where the axes of the body frame are
  aligned with the axes of the local NED frame: X points north, Y points east, Z points down.

- Matrices are represented in the row-major order.
