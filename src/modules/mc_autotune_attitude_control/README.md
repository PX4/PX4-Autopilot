# Multicopter autotune response verification

Autotune runs in PX4. QGroundControl starts it and polls the existing MAVLink
command progress. MC and FW modules accept commands only for the active vehicle
type, outside a VTOL transition.

## Selecting the experimental method

Standard Autotune remains the default. This build-time option includes one
implementation, so the experimental observer does not consume FLASH or heap
on standard builds. QGroundControl uses the same start command for either one.

Build the experimental SITL configuration with:

```sh
make px4_sitl_autotune-experimental
```

For a board with `CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL=y`, enable
`CONFIG_MC_AUTOTUNE_EXPERIMENTAL=y` in its
`.px4board` configuration. The target must have enough FLASH and heap for the
additional implementation. There is no runtime method selector.

The SITL test configuration enables the experimental implementation and runs
both implementations' lifecycle tests in separate executables. The following
acceptance rule and timing parameters describe the experimental method.

## Acceptance rule

A small additive torque measures the vehicle with its **existing gains**.
For each axis, RLS trains a candidate during two periods. The candidate is then
frozen. Two different excitation phase patterns each provide four measurement
periods, after two settling periods. No verification sample trains the candidate.

The verifier estimates the closed-loop transfer matrices from actual excitation
to torque, angular velocity and angular acceleration. It checks the proposed
controller replacement using these measurements, including axis coupling, the
attitude loop, the discrete PI controller and the yaw output filter. A conservative
lower bound on the Hermitian part of the return-difference matrix must exceed
0.2 after subtracting an empirical repeat/phase discrepancy envelope. The repeat
multiplier is 5.841 (Student t, three degrees of freedom); this is an engineering
uncertainty allowance, not a certified probability bound.

The full candidate is checked first. If rejected, the verifier tries unchanged
attitude gains, then half and quarter of the rate-gain change. Every alternative
must pass the same check. A negligible update does not count as success.

This is a finite-band empirical screening rule. It does not prove stability
between all measured frequencies or outside the measured operating condition,
and does not guarantee an optimal tune or increased bandwidth.

## Timing and failure behavior

- `MC_AT_PERIOD`: initial period, 8 s by default, configurable from 4 to 128 s.
  A complete three-axis measurement takes about `36 * period` seconds plus
  pauses. Insufficient low-frequency coverage doubles the period and restarts
  measurement. Insufficient high-frequency coverage rejects the candidate.
- `MC_AT_TIMEOUT`: total measurement budget, 600 s by default, configurable up
  to 14400 s. Retries use the same budget. For a 128 s period, one complete
  measurement alone requires over 4608 s. Set both parameters before starting.
- `MC_AT_SYSID_AMP`: relative torque excitation strength. Excessive tilt or
  unsuccessful allocation halves the amplitude and restarts that axis.
- Gains remain unchanged until verification succeeds. `MC_AT_APPLY=1` retains
  the existing apply-after-disarm behavior and ordinary parameter autosave.
- Mode changes, pilot input, disarm, parameter changes, invalid data and lost
  response data abort measurement. The rate controller independently expires
  the torque command after 100 ms and rejects it in an incompatible flight mode.
- Repeated timestamps do not add observations or train RLS a second time.

The supported experiment is Position-mode hover with no rate feedforward or
battery torque scaling. With nonzero attitude reference feedforward, attitude
gains remain unchanged. Existing gain bounds still apply. Very slow, noisy or
poorly excited vehicles can be rejected; increasing time alone cannot guarantee
acceptance. Vehicle mass is not used as a proxy for its dynamics.

The streaming observer uses approximately 35 KiB of heap storage. The rate loop
publishes synchronous measurement data; the frequency analysis runs in the
Autotune work item.
