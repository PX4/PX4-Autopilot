# Offboard Body-Frame Acceleration: Yaw-Only Rotation

| | |
|---|---|
| **Status** | Since v1.13, ported to v1.17. |
| **Code** | `handle_message_set_position_target_local_ned()` in `src/modules/mavlink/mavlink_receiver.cpp` |

## Summary

Upstream PX4 rotates offboard body frame velocity commands into NED by **yaw only**.

However for offboard acceleration commands it uses the **full attitude**.

This patch changes how PX4 handles acceleration commands to match velocity.

## Why

SI2 has its own pose estimate, which does not depend on PX4. In fact the two might disagree.

Therefore we need to send commands that do not depend on SI2 & PX4 sharing the same world frame.

We use the **FRDY frame** to do this. This is a drone's FRD frame but with roll = pitch = 0.

Because SI2 & PX4 use their own attitude estimates to form FRDY, the differences in north cancel.

## Deviation from MAVLink Spec

SI2 sends acceleration commands via the deprecated `MAV_FRAME_BODY_NED` MAVLink frame.

This frame is defined as NED for position and FRD for velocity and acceleration.

For body frame setpoints, PX4 only accepts offboard commands in the frame `MAV_FRAME_BODY_NED`.

Therefore this patch deviates from the MAVLink spec (acceleration now in FRDY instead of FRD).

Upstream PX4 is also inconsistent with MAVLink (velocity in FRDY, acceleration in FRD).

## Notes for porting new PX4 Versions

MAVLink has `MAV_FRAME_LOCAL_FRD` which is the correct MAVLink frame for FRDY. PX4 does not yet accept it.
