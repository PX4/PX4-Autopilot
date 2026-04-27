---
applyTo: "src/modules/mc_*control*/**,src/modules/fw_*control*/**,src/modules/flight_mode_manager/**,src/lib/rate_control/**,src/lib/npfg/**,src/modules/vtol_att_control/**"
---

# Control Review Guidelines

In addition to the core code review guidelines:

- Phase margin: output filters consume margin for no benefit; prefer adjusting gyro/d-gyro cutoffs
- Check for circular dependencies: sensor data feeding back into its own control loop (e.g., throttle-based airspeed in TECS)
- NaN propagation in flight-critical math; check `PX4_ISFINITE` before magnitude checks
- Prefer proper setpoint smoothing over controller output filtering (setpoint generation vs output-stage hacks)
- Check yaw control edge cases: heading lock, drift, setpoint propagation
- Verify flight task inheritance chain uses the correct base class for desired behavior
- Control allocation: verify actuator function ordering and motor index mapping
