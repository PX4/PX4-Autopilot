<Redirect to="../flight_controller/autopilot_discontinued" />

<!--
# CUAV X7 Flight Controller (Discontinued)

DOC REMOVED: 202603
-->

## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-12 support [DShot](../peripherals/dshot.md).
- Outputs 13-14 do not support DShot.
- Outputs 1-7, 9-12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 14 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)
- Outputs 9-12 in group3 (Timer1)
- Outputs 13-14 in group4 (Timer12)

All outputs within the same group must use the same output protocol and rate.
