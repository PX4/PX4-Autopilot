<Redirect to="../flight_controller/autopilot_discontinued" />

<!--
# ModalAI Flight Core v1 (Discontinued)

DOC REMOVED: 202603
-->

## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
