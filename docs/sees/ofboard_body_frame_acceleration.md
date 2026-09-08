# Offboard Body-Frame Acceleration: Yaw-Only Rotation

This note explains the Sees change to how PX4 handles acceleration setpoints sent in
`MAV_FRAME_BODY_NED`. It covers the coordinate frames involved, the inconsistency in
upstream PX4, and the fix.

- **v1.13:** `28e74b0b86` — "Interim fix for offboard control accel frame issue (#84)", `v1.13.1_dev`
- **v1.17:** `4816416854` — "fix: uses yaw-only rotation for offboard acceleration, matching velocity handling (#102)"
- **Code:** `MavlinkReceiver::handle_message_set_position_target_local_ned()` in
  [`src/modules/mavlink/mavlink_receiver.cpp`](../../src/modules/mavlink/mavlink_receiver.cpp)
- **Upstream bug:** [PX4-Autopilot#23255](https://github.com/PX4/PX4-Autopilot/issues/23255)

---

## 1. Background

### Coordinate frames

A coordinate frame is the set of axes that a number is measured against. A command such
as "accelerate at 2 m/s² in x" means nothing until you say which way x points.

The axis names tell you where each axis points:

| Name | Axes | Fixed to |
|---|---|---|
| **NED** | x = North, y = East, z = Down | The Earth. x points north whichever way the drone faces. |
| **FRD** | x = Forward, y = Right, z = Down | The vehicle. x points out of the nose. |

In both conventions, positive z points **down**. Climbing is therefore a *negative* z velocity.

### Attitude

Attitude is the vehicle's orientation, described by three angles:

- **Yaw** (heading): which compass direction the nose points.
- **Pitch**: nose tipped up or down.
- **Roll**: tipped left or right.

### Why tilt matters for multicopters

A multicopter can only push "up" relative to its own body. To accelerate forward it has
to pitch nose-down so that part of its thrust points forward. So **whenever a multicopter
accelerates, it is tilted**, often by 10–30°. This is what makes the choice of frame
matter.

---

## 2. The MAVLink frames involved

Definitions are taken from
[`common.xml`](../../src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml).

### `MAV_FRAME_LOCAL_NED` (1)

> Local coordinate frame, Z-down (x: North, y: East, z: Down).

Earth-fixed. Heading and tilt have no effect.

### `MAV_FRAME_BODY_FRD` (12)

> FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down) with an
> origin that travels with vehicle.

The axes are fixed to the airframe and follow **yaw, pitch and roll**.

- *Forward* means out of the nose. If the nose is pitched down, "forward" points partly
  into the ground.
- *Down* means out of the belly, which is also tilted.

### `MAV_FRAME_LOCAL_FRD` (20)

> FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to
> earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.

The axes follow **yaw only** and stay level with the ground.

- *Forward* means horizontal, in the direction the nose is facing, even while the vehicle
  is tilted.
- *Down* means straight down toward the Earth.

### `MAV_FRAME_BODY_NED` (8), deprecated

> Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as
> MAV_FRAME_BODY_FRD when used with velocity/acceleration values.

This is the frame MAVSDK's body-frame offboard API sends, and it is the frame that
reaches the code in question. By the spec, velocity and acceleration in this frame should
be interpreted as `BODY_FRD`, meaning fully tilted.

### Picture

A drone facing north-east (yaw 45°), pitched 20° nose-down while accelerating:

```
          BODY_FRD                          LOCAL_FRD

      nose tilted down               "forward" stays level
        \                            ───────────►  forward
         \  forward
          ▼ (points into ground)     │
     ┌───────┐                       ▼  down (toward Earth)
     │ drone │ tilted 20°
     └───────┘
          \  "down" (out of belly, also tilted)
```

In `LOCAL_FRD`, "forward" is horizontal toward the north-east. In `BODY_FRD`, "forward" is
toward the north-east and 20° below the horizon.

---

## 3. The inconsistency in upstream PX4

When `SET_POSITION_TARGET_LOCAL_NED` arrives with `coordinate_frame == MAV_FRAME_BODY_NED`,
upstream PX4 converts the body-frame setpoints to NED as follows:

| Field | Upstream rotation | Effective frame |
|---|---|---|
| Velocity | yaw only | `LOCAL_FRD` (level, heading-aligned) |
| Acceleration | full attitude `R` (yaw, pitch, roll) | `BODY_FRD` (tilted body) |

Upstream code (PX4 v1.13 and v1.17 alike):

```cpp
// velocity: yaw only
const float yaw = matrix::Eulerf{R}(2);
setpoint.velocity[0] = cosf(yaw) * velocity_body_sp(0) - sinf(yaw) * velocity_body_sp(1);
setpoint.velocity[1] = sinf(yaw) * velocity_body_sp(0) + cosf(yaw) * velocity_body_sp(1);
setpoint.velocity[2] = velocity_body_sp(2);

// acceleration: full attitude
const matrix::Vector3f acceleration_setpoint{R * acceleration_body_sp};
acceleration_setpoint.copyTo(setpoint.acceleration);
```

So a single message carries a velocity and an acceleration that are in **different
frames**. The two frames only agree when the vehicle is level, and a multicopter is not
level while it is accelerating.

### Consequences

**1. Horizontal acceleration leaks into vertical.**
Command a forward acceleration `a` while pitched nose-down by θ. The full-attitude rotation
gives, in the heading frame:

```
[ a·cos θ,  0,  a·sin θ ]     (forward, right, down)
```

| Tilt θ | Forward (fraction of `a`) | Downward (fraction of `a`) |
|---|---|---|
| 10° | 0.98 | 0.17 |
| 20° | 0.94 | 0.34 |
| 30° | 0.87 | 0.50 |

At 20° tilt, about a third of the requested forward acceleration becomes a downward
acceleration, and the vehicle gets about 6% less forward acceleration than it asked for.
Roll does the same thing on the lateral axis.

**2. The command depends on the vehicle's response to it.**
The acceleration setpoint drives the tilt, and the tilt changes how the next acceleration
setpoint is rotated. So the NED feedforward that reaches the position controller depends
on how the vehicle responded to the previous command. A controller should not have that
circular dependency. Yaw is not driven by translational acceleration, so a yaw-only
rotation does not have this problem.

**3. Velocity and acceleration feedforward disagree.**
If a companion computer sends a matching velocity and acceleration (for example, from a
trajectory generator), PX4 receives an acceleration that does not point along the velocity
it was sent with.

---

## 4. The fix

The Sees change rotates acceleration by **yaw only**, exactly as upstream already does for
velocity:

```cpp
// sees updated code
const float yaw = matrix::Eulerf{R}(2);
setpoint.acceleration[0] = cosf(yaw) * acceleration_body_sp(0) - sinf(yaw) * acceleration_body_sp(1);
setpoint.acceleration[1] = sinf(yaw) * acceleration_body_sp(0) + cosf(yaw) * acceleration_body_sp(1);
setpoint.acceleration[2] = acceleration_body_sp(2);
```

x and y are rotated about the vertical axis by the vehicle's heading. z is passed through
unchanged, so "down" always means toward the Earth.

After the fix, both velocity and acceleration in `MAV_FRAME_BODY_NED` are interpreted as
**`MAV_FRAME_LOCAL_FRD`** (level, heading-aligned).

| Field | After fix | Effective frame |
|---|---|---|
| Velocity | yaw only (unchanged) | `LOCAL_FRD` |
| Acceleration | yaw only | `LOCAL_FRD` |

The v1.17 port is logically identical to v1.13 #84. The only differences are in the
surrounding upstream code (for example, `copyTo` instead of assigning NaN per element).

---

## 5. Is it correct?

**As a control design: yes.**
For controlling a multicopter, a gravity-aligned heading frame is almost always what
"forward / right / down" means. Velocity and acceleration are now in the same frame, the
vertical leakage is gone, and the command no longer depends on tilt. `Eulerf{R}(2)` is
only poorly defined near ±90° pitch, which is not reachable in multicopter offboard flight.
The velocity path relies on the same assumption.

**Against the MAVLink spec: no, deliberately.**
The spec says `BODY_NED` velocity and acceleration mean `BODY_FRD` (fully tilted). Strictly,
upstream's *acceleration* handling was the spec-compliant half and its *velocity* handling
was the non-compliant half. This fix chooses consistency with velocity, and with what
operators expect, over spec compliance. The in-code comment says so.

**Implications for anyone sending commands to this firmware:**
acceleration sent in `MAV_FRAME_BODY_NED` is interpreted as level and heading-aligned, not
as tilted-body. Anything that expects strict `BODY_FRD` semantics will be wrong on this
firmware.

---

## 6. Scope and caveats

- **Affected path:** only MAVLink `SET_POSITION_TARGET_LOCAL_NED` with
  `MAV_FRAME_BODY_NED`. Setpoints sent in `MAV_FRAME_LOCAL_NED`, or published directly as
  `trajectory_setpoint` over uXRCE-DDS / ROS 2, are not affected.
- **Upstream type-mask quirk (not introduced by this fix):** `ignore_velocity` and
  `ignore_acceleration` are true if *any* one of the three axis bits is set. Ignoring only
  AZ therefore drops all three acceleration axes, and the per-axis `? 0.f :` ternaries
  inside the block never take effect. Do not rely on partially masked body-frame
  acceleration.
- **Long-term fix:** add proper handling for `MAV_FRAME_LOCAL_FRD` (yaw only) and
  `MAV_FRAME_BODY_FRD` (full attitude) as separate frames, restore spec behaviour for
  `BODY_NED`, and expose the frames through MAVSDK. Until then this interim fix should be
  carried forward on each PX4 upgrade.
