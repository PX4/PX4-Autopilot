# Timestamp domain bug

Incomming mavling message has wrong timestamp:
mavlink_ranging_beacon_t.time_usec = 650880495394
while system time is 1039768000
This is Copilot generated message.


EKF2::UpdateRangingBeaconSample() uses `timestamp_sample`, not `timestamp` field:
```c++
rangingBeaconSample sample{
			.time_us = ranging_beacon.timestamp_sample,
```

## Root cause (confirmed)

`MavlinkReceiver::handle_message_ranging_beacon` (mavlink_receiver.cpp:2743) copies
`beacon_pos.time_usec` straight into `ranging_beacon.timestamp_sample`, with no
`_mavlink_timesync.sync_stamp()` conversion. Every other external-aiding handler in
that file (mocap, vision odometry, landing_target_pose, gps_input target) does call
`sync_stamp()` to convert the sender's clock into the local `hrt_absolute_time()`
domain before publishing.

Downstream, EKF2 trusts the uORB producer to have already normalized the timestamp:
- `EKF2::UpdateRangingBeaconSample()` forwards `timestamp_sample` verbatim into
  `rangingBeaconSample.time_us`, no conversion.
- `EstimatorInterface::setRangingBeaconData()` and
  `Ekf::controlRangingBeaconFusion()` compare `sample.time_us` directly against
  the ring buffer's newest sample and `imu_delayed.time_us` — both hrt-domain.

So the sender-domain timestamp (Unix epoch or sender boot time, per the MAVLink
spec: "Timestamp (UNIX Epoch time or time since system boot)...") never gets
reconciled with PX4's own hrt clock. Result: `pop_first_older_than()` / the
min-interval check compare mismatched clock domains, so beacon fusion can starve
or misbehave depending on which side's clock is larger.

Introduced whole-cloth in c2607941 ("feat(mavlink): add ranging beacon parser and
uORB message", Marco Hauswirth, 2026-03-31) — never had a sync_stamp() call, not a
later regression.

**Fix**: `ranging_beacon.timestamp_sample = _mavlink_timesync.sync_stamp(beacon_pos.time_usec);`
matching the pattern used by every sibling handler.

---

# EKF failsafe

Failsafe activated: switching to Land
- Navigation error: No valid position estimate
- Navigation error: No valid global position estimate
- No manual control input

check src/modules/commander/HealthAndArmingChecks/checks/modeCheck.cpp
Local position error is greater than threshold: lpos.eph, lpos_eph_threshold
Global position is still valid.
