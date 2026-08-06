## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 2 groups:

- Outputs 1-4 in group1 (PWM2)
- Outputs 5-8 in group2 (PWM4)

All outputs within the same group must use the same output protocol and rate.