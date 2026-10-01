# GNSS failover SIH tests (step 7 of #28813)

Draft placeholder: this file is the plan and is deleted when the work lands. Stacked on the step 6 PR, so the tests can assert the switch event and `gnss_fusion_state`. Full plan: [failover_validation.md](https://github.com/dakejahl/botenook/blob/main/gnss/failover_validation.md).

## Goal

Show in SIH, in CI, that a failed receiver hands over to the good one and EKF2 follows it, for every failure the checks can see.

## Pass criteria

For one failure of the selected receiver, armed, in the air, in a position-controlled mode, with a healthy standby:

- The selection changes once, to the standby.
- Every EKF2 instance resets horizontal position once, and height if GNSS is the height reference. The reset delta equals the offset between the receivers after lever arms.
- Position stays valid, no failsafe triggers other than `gnss_lost` when `SYS_HAS_NUM_GNSS` asks for it, and the mode doesn't change. In Hold the vehicle doesn't move.
- The selection doesn't return to the recovered receiver before disarm.
- One switch event names the reason, and `gnss_fusion_state` shows why GNSS wasn't fused in between.

## Scope

1. **Second SIH receiver**: an explicit enable, independent noise, and a per-receiver position and height bias, so that a switch produces a reset delta.
2. **Injection modes** in `lib/failure_injection` `process_gnss`, so every driver gets them: `wrong` also overrides eph, epv, speed accuracy, satellite count and spoofing state from new `SYS_FAIL_GPS_*` parameters (0 leaves the field unchanged), and `slow` publishes one sample in N.
3. **Heading follows injection**: `process()` on every `sensor_gnss_relative` publisher (`gps`, `septentrio`, DroneCAN). For a moving-base rover (`SENS_GNSSn_HDG = 1`), the hub drops its heading while the receiver in the base slot is silent. SIH publishes a simulated relative heading from ground-truth yaw.
4. **Cases** in `test/mavsdk_tests` with `sih-sitl.json` overrides: data timeout, failed check, failure on a mission leg, height reset, no return while armed, standby failure, total loss and recovery, 1 Hz toggling, ranked selection on failure, accuracy and update rate, and heading loss and recovery.
5. **Tooling**: MAVSDK C++ scenario functions shared by the catch2 cases and a companion `gnss_failover` binary for the step 10 flight test, and a Python (pyulog) log report in `Tools/` that grades SIH and flight logs against the pass criteria. `docs/en/debug/failure_injection.md` describes the new modes and the tool.

## Out of scope

- The flight test: step 10, after step 9.
- A selected receiver that is wrong but passes its checks (stuck, slow drift).
- Multiple EKF2 instances.
