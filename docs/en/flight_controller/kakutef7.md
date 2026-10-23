<Redirect to="../flight_controller/autopilot_discontinued" />

<!--
# Holybro Kakute F7 (Discontinued)

DOC REMOVED: 202603
-->

## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer1)
- Output 5 in group3 (Timer8)
- Output 6 in group4 (Timer5)

All outputs within the same group must use the same output protocol and rate.
