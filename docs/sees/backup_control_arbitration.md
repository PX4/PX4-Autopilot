# Backup Control Arbitration

## Background

Standard PX4 radio failover behaviour is a _static_ configuration - we can choose to failover to either Backup GCS (BGCS - QGroundControl) or R/C Safety Console (RCSC). We wish to change this behaviour as follows:

- Treat BGCS and RCSC as equivalent
- Implement the ability to hand over control between BGCS and RCSC _dynamically_ during flight

Why do we need it: A single flight can have both VLOS and BVLOS stages. RCSC serves as the failover control console in VLOS operations. BGCS is required for BVLOS operations. Since we can have both VLOS and BVLOS segments in a flight, a _static_ configuration for a single failover mechanism is insufficient.

## Glossary of terms

- BGCS: Backup GCS (`QGroundControl`)
- RCSC: Radio controlled safety console (handheld R/C controller)
- FC: Flight controller. Autopilot hardware running PX4 firmware

## Requirements

1. RCSC operator can toggle a switch to either take manual control or handover control to BGCS
2. BGCS operator can toggle a switch to either take manual control or handover control to RCSC
3. RCSC has priority
    1. If both BGCS and RCSC attempt to take control, RCSC gets control authority
4. FC acts on stick (joystick) inputs from whoever has control authority.
    1. Stick inputs from non-active controller is masked
5. FC listens to following switch (toggle) inputs from both controllers at all times: kill switch, mode switch, arbitration control switch
6. If RCSC switches to any _mode_ (not just position mode - decouples from onboard mode), it automatically gains control authority

> Note: _MODE_ switch has three positions assigned the following: [position hold] - [altitude hold] - [position hold]. This makes it easy to signal a change in the switch position, even if it's a position->position transition

> Note: Kill switch behaviour on RCSC: Kill switch is a spring loaded toggle on the RCSC. Switch in _kill_ position immediately disarms the PWM motor outputs, but it must be held continuously for 10 seconds to make the action permanent. Letting go before 10 seconds reverses the action, and filters for unintended triggers. On BGCS and SI2, kill means kill: a disarm message is send via mavlink.

### Unresolved

- What happens when BGCS if offline and RCSC hands it control authority?
- Confirm that when neither are in control, the drone is in position hold mode.

## Verificaton

See [Test Plan](test_plan.md)
