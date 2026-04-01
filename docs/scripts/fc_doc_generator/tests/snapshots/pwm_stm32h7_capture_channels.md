## PWM Outputs {#pwm_outputs}

This flight controller supports up to 16 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 9-16 do not support DShot.
- Outputs 1-7 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 16 outputs are in 5 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)
- Outputs 9-11 in group3 (Timer1)
- Outputs 12-14 in group4 (Timer8)
- Outputs 15-16 in group5 (Timer12)

All outputs within the same group must use the same output protocol and rate.