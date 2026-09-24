<Redirect to="../flight_controller/autopilot_discontinued" />

<!--
# NXP RDDRONE-FMUK66 FMU (Discontinued)

DOC REMOVED: 202603
-->

## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (FTM0)
- Outputs 5-6 in group2 (FTM3)

All outputs within the same group must use the same output protocol and rate.
