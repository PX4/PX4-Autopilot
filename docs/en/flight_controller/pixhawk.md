<Redirect to="../flight_controller/autopilot_discontinued" />

<!--
# 3DR Pixhawk 1 Flight Controller (Discontinued)
Doc removed 202604
-->

## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
