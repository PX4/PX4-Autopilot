## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6, 9 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6, 9 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)
- Output 9 in group4 (Timer1)

All outputs within the same group must use the same output protocol and rate.