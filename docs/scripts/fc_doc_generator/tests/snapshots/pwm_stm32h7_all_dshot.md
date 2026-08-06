## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer8)

All outputs within the same group must use the same output protocol and rate.