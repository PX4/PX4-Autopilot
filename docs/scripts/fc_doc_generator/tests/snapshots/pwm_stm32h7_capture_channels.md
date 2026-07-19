## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-7 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 8 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.