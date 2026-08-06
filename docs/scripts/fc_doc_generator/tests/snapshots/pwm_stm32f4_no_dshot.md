## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 3 groups:

- Outputs 1-2 in group1 (Timer1)
- Outputs 3-4 in group2 (Timer4)
- Outputs 5-6 in group3 (Timer5)

All outputs within the same group must use the same output protocol and rate.