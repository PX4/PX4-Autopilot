<Redirect to="../flight_controller/autopilot_discontinued" />

<!--
# Omnibus F4 SD (Discontinued)

DOC REMOVED: 202603
-->

## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 4 outputs are in 2 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)

All outputs within the same group must use the same output protocol and rate.
