# Quad-X Turtle Recovery

Turtle recovery lets a pilot turn an overturned quadcopter upright using its own motors while the vehicle is disarmed.
The pilot selects the recovery direction with the roll and pitch sticks.
The DShot driver sends temporary reverse-direction commands to the mapped motors, drives the selected motors for a bounded interval, then sends normal-direction commands before releasing its arming interlock.

This feature defaults disabled.
It deliberately permits motor output while disarmed when its recovery conditions are met.

::: info
With `TRTL_ESC=1`, the DShot driver restores normal motor direction at startup, including when recovery is disabled or RC input is unavailable.
Normal and actuator-test output remain blocked until restoration completes.
See [Startup Direction Restoration](#startup-direction-restoration) for the configuration and hardware requirements.
:::

## Supported Configuration

The controller accepts a conventional four-corner Quad-X with four fixed vertical motor axes, logical Motors 1–4, and all four motors on one DShot driver instance.
The configured corner map must agree with the control-allocation geometry and the physical output functions.
Plus frames, tilted motors, VTOL, more than four motors, mixed output backends and reversible/3D throttle configurations are not supported.

The ESC firmware must support temporary relative DShot direction commands 20 (normal) and 21 (reversed), including repeated commands while stopped.
Ordinary DShot support alone does not establish this compatibility.
Set `TRTL_ESC=1` only after verifying that the ESC firmware supports this behaviour.
The parameter does not identify or detect compatible ESCs automatically.
It also opts the configured outputs into startup direction restoration, independently of `TRTL_EN`.
The driver latches the ESC profile on its first successful parameter read.
Recovery does not save ESC settings.

Input comes through PX4's calibrated RC pipeline, with a valid RC-source manual setpoint and fresh RC channel data.
Joystick `MANUAL_CONTROL` input alone does not satisfy the RC-source requirement.

## Configuration

Remove propellers for output mapping, direction checks and initial testing.
First verify ordinary motor mapping and spin direction using [Actuator Configuration](../config/actuators.md).
Configure recovery in _QGroundControl_ with the `TRTL_*` parameters.

| Parameters                                          | Purpose                                                                        |
| --------------------------------------------------- | ------------------------------------------------------------------------------ |
| `TRTL_EN`, `TRTL_ESC`                               | Enable recovery entry and opt compatible ESCs into direction restoration.      |
| `TRTL_CH`, `TRTL_POL`                               | Select the one-based calibrated RC channel and switch polarity.                |
| `TRTL_FL`, `TRTL_FR`, `TRTL_RL`, `TRTL_RR`          | Assign logical Motor 1–4 to front-left, front-right, rear-left and rear-right. |
| `TRTL_ANGLE`, `TRTL_EXIT`                           | Set entry and automatic-exit tilt angles, measured from upright.               |
| `TRTL_MIN`, `TRTL_MAX`                              | Set starting and maximum recovery motor commands.                              |
| `TRTL_RAMP`, `TRTL_TIME`                            | Set upward ramp time and maximum driving time.                                 |
| `TRTL_PRE`, `TRTL_POST`, `TRTL_SETTLE`, `TRTL_COOL` | Set stop, reversal settling and restoration cooldown intervals.                |
| `TRTL_DB`, `TRTL_PWR`, `TRTL_HOLD`                  | Set stick deadband, proportional power and deliberate activation hold.         |

Motor corner parameters refer to logical motor numbers, not connector numbers.
Determine corners looking down at the upright aircraft with its nose forward.
All four values must be distinct; zero means unconfigured.

The recovery channel must not also control roll, pitch, yaw, throttle, arming, kill, flight mode or other critical mapped functions.
Sharing with a generic AUX function is allowed.
After applying polarity, values above 0.7 mean ON and values below −0.3 mean OFF.
The middle position cancels recovery but does not qualify as OFF for another attempt.

Motor command values are PX4 DShot units, not RPM, measured thrust or percentage power.
Tune motor commands and timing for the aircraft's propulsion system.
`TRTL_MIN` must not exceed `TRTL_MAX`, `TRTL_RAMP` must not exceed `TRTL_TIME`, and `TRTL_EXIT` must be at least 10 degrees below `TRTL_ANGLE`.
An entry angle of 85 degrees permits a nominal 90-degree side-on start.

Apply recovery tuning changes while disarmed, with the recovery switch OFF and the recovery sequence idle.
Keep the ESC profile and motor assignments unchanged throughout a powered ESC session.
Changing `TRTL_ESC`, motor corner assignments or physical output functions requires a complete FC and ESC power cycle before further recovery use.
A driver restart or FC-only reboot is not sufficient after such edits.
Inspect `dshot status` to confirm the active configuration, output map, state and readiness.

## Startup Direction Restoration

With a compatible ESC profile selected, the driver starts with direction readiness unknown, including when `TRTL_EN=0`.
It blocks normal and actuator-test throttle while restoration is pending.
While disarmed and with valid configuration and hardware conditions, it sends STOP for the configured dwell, a bounded normal-direction command train, then waits for cooldown.
Direction commands are not sent while armed.
An existing external command transaction can finish before restoration takes ownership; new conflicting configuration requests are rejected while restoration is pending.

Startup restoration does not require RC input or switch-OFF qualification.
Other hardware and safety conditions, including an active failsafe, can still prevent it from progressing.
Invalid motor geometry or recovery configuration also leaves readiness blocked.
An RC channel conflict prevents recovery entry but does not by itself prevent startup restoration.

After startup restoration completes, normal arming uses the existing PX4 checks.
Ordinary rearming does not repeat the direction sequence.
A new turtle attempt still requires fresh, valid switch-OFF input followed by ON.

## Operation

### Pilot Sequence

Assign the recovery radio channel and switch polarity during setup.
The following sequence assumes startup normalisation has completed and the aircraft meets the configured entry tilt.

1. Disarm the aircraft, lower throttle fully and leave the recovery switch OFF.
2. Hold roll or pitch toward the desired recovery direction, then switch recovery ON and maintain the entry conditions for `TRTL_HOLD`.
   Direction input can also be applied after switching ON; it is not itself an entry prerequisite.
3. Keep throttle low while the selected motors drive the flip.
   Recovery ends when the aircraft comes within `TRTL_EXIT` of upright or the drive time expires; switching OFF also ends the attempt.
4. Return the recovery switch OFF and keep throttle low while the stop, normal-direction restoration and cooldown sequence completes.
5. After valid continuous OFF input qualifies release, arm normally if PX4's other arming checks pass.
   If the aircraft is still overturned, leave the switch OFF through release before switching ON for another attempt.

Enabling recovery does not arm the aircraft.
Commander and the recovery controller exclude simultaneous flight and recovery ownership.

### Software Sequence

1. Validate disarmed state, low throttle, fresh RC and attitude input, output mapping, entry tilt and a qualified OFF-to-ON request.
   Acquire recovery ownership and send STOP while enforcing `TRTL_HOLD`.
2. Continue STOP for `TRTL_PRE`, then send temporary reverse-direction commands to all four mapped ESCs.
   Wait `TRTL_SETTLE` before entering the drive phase.
3. Start the bounded `TRTL_TIME` drive window.
   Power the selected pair or corner according to current roll/pitch input, `TRTL_PWR` and `TRTL_RAMP`; unselected motors receive STOP.
4. On completion or a controlled cancellation, send STOP for `TRTL_POST`, send normal-direction commands to all four mapped ESCs and wait `TRTL_COOL`.
   Recovery ownership remains held throughout this sequence.
5. After cooldown, require valid RC, low throttle and a continuous OFF interval of at least 300 ms before releasing ownership.
   This OFF interval is separate from `TRTL_COOL`; OFF time during the earlier restoration phases does not count towards it.

Changing the selected motor group restarts its output ramp without restarting the overall drive deadline.

### Interruptions And Other Inputs

| Event | Response |
| --- | --- |
| Roll/pitch centred within the deadband | Send STOP while remaining in the drive phase. The original drive deadline continues; renewed stick input can resume drive before timeout. Centring alone does not begin restoration. |
| Switch leaves ON | Cancel and enter the stop/restoration sequence. The middle position cancels but does not qualify as OFF for release or retry. |
| RC input becomes invalid or throttle rises above the low threshold | Cancel recovery drive. Restoration proceeds when its hardware conditions permit; valid RC, low throttle and OFF are required for release. |
| Aircraft reaches the configured exit tilt or drive time expires | Stop driving and begin restoration. The switch must return OFF before a new attempt. |
| Recovery disabled during an owned sequence | Stop drive while retaining ownership through restoration and OFF release. |
| Hardware condition prevents restoration | Keep output stopped and delay or restart restoration. A transaction timeout can latch a fault. |
| Mapping changes during ownership or a restoration fault is latched | Keep output stopped and recovery ownership blocked; ordinary switch-OFF release is insufficient. |

A full FC reset interrupts execution and loses the in-memory recovery sequence.
If the ESCs remain powered, their temporary direction settings can persist.
The startup behaviour and operating requirements for that case are described under Diagnostics And Faults below.

## Diagnostics And Faults

`dshot status` reports capability, pending direction restoration, configuration validity, readiness, phase, cancellation reason and the resolved physical output map.
It also reports attempts and direction/throttle frame counters.
These counters record software commands sent; they do not acknowledge that an ESC accepted a direction change.

Loss of the selected RC channel invalidates recovery input and requires fresh OFF qualification.
Callback gaps longer than 100 ms invalidate OFF dwell, and interrupt an active attempt.
If the driver is destroyed while it owns recovery outputs, the ownership fault persists through driver recreation within the same FC boot.
A full FC reset clears that in-memory state and starts a new normal-direction sequence when the persisted compatible ESC profile is enabled.
The driver does not reconstruct the interrupted transaction or resume turtle driving automatically.
Independent ESC resets are not detected automatically.

::: warning
Startup restoration after an FC-only reset requires the same mapped ESCs to remain powered, connected and responsive through the stop dwell and command train.
Changing persisted capability or output assignments before an FC reset can lose the information needed to restore the previous ESC set.
Keep those settings stable, or power-cycle both FC and ESCs before further recovery use.
If direction state is uncertain, power-cycle both the FC and ESCs and verify actual normal motor directions before ordinary arming.
:::
