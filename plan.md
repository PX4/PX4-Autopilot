# GNSS reporting (step 6 of #28813)

Draft placeholder: this file is the plan and is deleted when the work lands. Stacked on #28798. Design: [RFC #28813](https://github.com/PX4/PX4-Autopilot/issues/28813), [pipeline_migration.md](https://github.com/dakejahl/botenook/blob/main/gnss/pipeline_migration.md) §2 (decisions 6, 8–10) and §4.

## Goal

Commander and the user learn from the new topics why GNSS is not fused and which receiver failed. The estimator-side GNSS reporting that predates those topics goes.

## Scope

1. **`gnss_fusion_state`**: EKF2 publishes on `estimator_status_flags` why the latest GNSS sample was or was not fused: fused, no data, unusable (`vehicle_gnss.usable` false), rejected by the innovation gate, above `EKF2_VEL_LIM`, or inactive (`EKF2_GPS_CTRL`, alignment, a declared GNSS fault).
2. **Commander reads the new topics**: pre-arm and in-flight GNSS messages come from `vehicle_gnss` (`usable`, `failed_checks`) and `sensors_status_gnss` instead of `estimator_status.gps_check_fail_flags`. The loss-reason event from #28873 takes its reason from `gnss_fusion_state` and names the receiver.
3. **Switch event**: one event per `vehicle_gnss.selection_count` change, with `selection_reason` and both receivers.
4. **Divergence in the hub**: `sensors_status_gnss.inconsistency` (horizontal distance to the selected receiver after lever arms) and `priority` (the preferred receiver). Commander's `gnss_lost` divergence test reads `inconsistency` instead of computing its own.
5. **Consumers gate on `usable`** instead of their own fix-type tests. HomePosition must; geofence, RemoteID, open_drone_id and the mag and baro calibrations are reviewed one by one.
6. **Removed**: `estimator_status.gps_check_fail_flags` and its `GPS_CHECK_FAIL_*` constants, once commander and HIGH_LATENCY2 read the new topics.

## Out of scope

- Per-receiver MAVLink GNSS messages, which wait for mavlink/mavlink#2146. `GPS_RAW_INT` and `GPS2_RAW` stay on fixed receivers.
- SIH tests that assert the switch event and `gnss_fusion_state`: step 7.

## Open

- Items 3 and 4 were planned for the selection but left out of #28798. They can move to their own PR if this one gets large.
