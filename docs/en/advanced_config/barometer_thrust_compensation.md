# Barometer Thrust Compensation

Propellers change the static pressure at the barometer sensor proportional to motor output.
This creates a thrust-dependent altitude error that can reach several metres on small vehicles with the barometer close to the propellers.
The direction and magnitude depend on sensor placement relative to the propellers.

PX4 can compensate for this error by applying a correction proportional to the vertical thrust setpoint:

```txt
corrected_baro_alt = raw_baro_alt + SENS_BARO_PCOEF * |thrust_z|
```

where `thrust_z` is the Z body-axis component of `vehicle_thrust_setpoint` (negative for upward thrust in FRD frame).

The compensation parameter [SENS_BARO_PCOEF](../advanced_config/parameter_reference.md#SENS_BARO_PCOEF) can be identified automatically during flight or manually from a flight log.

::: info
This feature compensates for _propwash-induced_ pressure error, which depends on motor output.
For _airspeed-induced_ static pressure error (due to vehicle forward motion), see [Static Pressure Buildup](../advanced_config/static_pressure_buildup.md).
:::

## Online Calibration (Recommended)

The online estimator identifies `SENS_BARO_PCOEF` automatically during flight and saves the result on disarm.

### How It Works

A complementary filter (CF) fuses barometer altitude with double-integrated accelerometer data at a very low crossover frequency (default 0.05 Hz).
The CF trusts the accelerometer for fast altitude changes and the barometer for slow drift, so the CF residual (baro minus accel prediction) isolates thrust-correlated pressure error while rejecting real altitude changes.

A Recursive Least Squares (RLS) estimator then fits the linear model `residual = K * thrust + bias` to identify the gain K.
Once the estimate converges (stable K, low variance, sufficient thrust excitation), K is locked and saved to `SENS_BARO_PCOEF` on disarm.

The estimator refines the parameter over subsequent flights — each flight corrects for whatever residual error remains after the previous calibration.

### Setup

1. Set [SENS_BAR_AUTOCAL](../advanced_config/parameter_reference.md#SENS_BAR_AUTOCAL) to **3** (enables both GNSS altitude calibration and thrust compensation).
2. Fly normally for at least 60 seconds with some altitude variation.
3. On disarm, the estimated `SENS_BARO_PCOEF` is saved automatically if the estimator converged.
4. Check convergence after flight:
   - In the console: `baro_thrust_estimator status`.
   - In a log: look at the `baro_thrust_estimate` topic — `converged` should be true.

The estimator uses several convergence gates before saving:

| Gate                 | Threshold                | Purpose                       |
| -------------------- | ------------------------ | ----------------------------- |
| Minimum flight time  | 30 s                     | Allow RLS to settle           |
| K variance (P[0][0]) | < 3.0                    | Parameter estimate is precise |
| Prediction error     | Low absolute or relative | Model fits the data           |
| Thrust excitation    | std > 0.05               | Enough signal to identify K   |
| K stability          | Stable for 10 s          | Estimate is not drifting      |
| Hold time            | 10 s                     | Convergence is sustained      |

::: tip
Altitude changes during hover provide the thrust excitation the estimator needs.
Constant-thrust hover with no altitude variation will not converge.
:::

### Parameters

| Parameter                                                                      | Default | Description                                                                                           |
| ------------------------------------------------------------------------------ | ------- | ----------------------------------------------------------------------------------------------------- |
| [SENS_BARO_PCOEF](../advanced_config/parameter_reference.md#SENS_BARO_PCOEF)   | 0.0     | Baro altitude correction per unit vertical thrust \[m\]. Identified by the estimator or set manually. |
| [SENS_BAR_AUTOCAL](../advanced_config/parameter_reference.md#SENS_BAR_AUTOCAL) | 1       | Bitmask: bit 0 = GNSS altitude offset, bit 1 = online thrust compensation. Set to 3 for both.         |
| [SENS_BAR_CF_BW](../advanced_config/parameter_reference.md#SENS_BAR_CF_BW)     | 0.1     | CF crossover frequency \[Hz\]. Lower = more conservative, higher = faster identification but noisier. |

### Soft Guards

The estimator pauses RLS updates (while keeping the CF running) when:

- Vertical speed exceeds 2 m/s
- Horizontal speed exceeds 5 m/s

This prevents high-speed flight dynamics from corrupting the estimate.
Once converged, the RLS is frozen entirely to prevent ground-effect contamination during landing.

## Manual Calibration

If you prefer not to use the online estimator, you can identify `SENS_BARO_PCOEF` from a flight log using a range sensor as ground truth.

1. Set `SENS_BARO_PCOEF` to 0 (disable existing compensation).
2. Fly a hover at 2-5 m AGL for at least 60 seconds with gentle altitude changes. A downward-facing range sensor must be installed.
3. Run the analysis script on the log:
   ```sh
   python3 Tools/baro_compensation/baro_thrust_calibration.py <path/to/log.ulg>
   ```
4. Apply the recommended `SENS_BARO_PCOEF` value.
5. Fly again and re-run the script to verify the compensation.

The script automatically selects an analysis mode based on available data:

| Mode                   | Data Required            | Output                                   |
| ---------------------- | ------------------------ | ---------------------------------------- |
| Estimator review       | Online estimator logged  | K convergence, residual analysis         |
| Full validation        | Estimator + range sensor | Cross-validation of online vs offline K  |
| Standalone calibration | Range sensor only        | Recommended PCOEF from least-squares fit |

## Interpreting Results

| Metric                   | Good  | Marginal  | Poor  |
| ------------------------ | ----- | --------- | ----- |
| Thrust correlation \|r\| | > 0.6 | 0.3 - 0.6 | < 0.3 |
| Model R^2                | > 0.3 | 0.1 - 0.3 | < 0.1 |
| Compensated \|r\|        | < 0.2 | 0.2 - 0.4 | > 0.4 |

- **Low R^2**: Thrust is not the dominant baro error source. Check for thermal drift, ground effect, or sensor placement issues.
- **Online/offline K disagreement > 2 m**: The estimator may need more excitation. Fly longer or with more altitude variation.

## See Also

- [Static Pressure Buildup](../advanced_config/static_pressure_buildup.md) — airspeed-induced barometer error
- [Compass Power Compensation](../advanced_config/compass_power_compensation.md) — analogous compensation for magnetometer
- [Sensor Thermal Compensation](../advanced_config/sensor_thermal_calibration.md) — temperature-induced sensor error
- [Using PX4's Navigation Filter (EKF2)](../advanced_config/tuning_the_ecl_ekf.md) — EKF2 height fusion configuration
