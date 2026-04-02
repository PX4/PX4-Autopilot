# Barometer Thrust Compensation Analysis

Post-flight analysis tool for barometer thrust compensation. Works with the
online `baro_thrust_estimator` module and/or a range sensor for ground truth.

## Background

Propwash from propellers changes the static pressure at the barometer sensor.
This creates a thrust-dependent altitude error: as thrust increases, the baro
reading shifts. The direction and magnitude depend on sensor placement relative
to the propellers.

The `vehicle_air_data` module compensates for this by applying a correction to
the barometer altitude before publishing:

```
corrected_baro_alt = raw_baro_alt + SENS_BARO_PCOEF * abs(thrust_z)
```

where `thrust_z` is the Z body-axis component (`vehicle_thrust_setpoint.xyz[2]`),
which is signed in PX4 (negative for upward thrust in FRD). The correction uses
its magnitude `|thrust_z|` in [0, 1]. Using the vertical thrust setpoint (rather
than individual motor outputs) provides correct behavior for both multicopter and
VTOL aircraft.

## How Calibration Works

### Online (Preferred)

The online estimator is disabled by default. Enable it by setting
`SENS_BAR_AUTOCAL` bit 1 (e.g. set to 3 for both GNSS cal and thrust comp).
The `baro_thrust_estimator` module identifies `SENS_BARO_PCOEF` automatically
during flight using an accel-baro complementary filter and RLS estimation.
Parameters are saved to flash on disarm once converged, and the estimator
continues to refine PCOEF over subsequent flights. No range sensor required.

### Offline (This Tool)

This tool identifies PCOEF from a flight log using a range sensor as ground
truth. Useful for:
- Validating online estimator results against ground truth
- Analyzing compensation effectiveness
- Initial calibration when the online estimator hasn't converged yet

## Prerequisites

- **Python packages**: `pyulog`, `numpy`, `matplotlib`

  ```bash
  pip install pyulog numpy matplotlib
  ```

## Analysis Modes

The tool automatically selects a mode based on what data is in the log:

### 1. Estimator Review (online estimator logged, no range sensor)

Shows what the online estimator did during the flight:
- K estimate convergence with uncertainty band
- Prediction error and thrust excitation
- Convergence status timeline
- Compensation effect (residual before/after correction)

### 2. Full Validation (online estimator + range sensor)

Cross-validates the online estimator against range-sensor ground truth:
- All estimator review plots (above)
- Side-by-side comparison of online vs offline compensation
- Scatter plots comparing raw, online-corrected, and offline-corrected error

### 3. Standalone Calibration (range sensor, no estimator)

Identifies PCOEF from baro vs range error:
- Cross-correlation delay analysis
- Recommended PCOEF value

## Usage

```bash
python3 baro_thrust_calibration.py <path/to/log.ulg> [--output-dir <dir>]
```

## Output

### Console

Summary including parameters found in the log, online estimator convergence
status, range-based error statistics, and cross-validation between online and
offline identification (when both are available).

### PDF Report (`<log_name>.pdf`)

Pages vary by mode. When the online estimator is present:

**Altitude Overview** — Baro observation, EKF altitude, distance sensor (if
available), and thrust over time.

**Estimator Convergence** — K estimate with variance band, error variance
and thrust excitation, convergence flags over time.

**Compensation Effect** — CF residual before/after applying estimated K,
scatter plots, and compensation statistics.

When a range sensor is present, additional pages show:

**Compensation Comparison** — Side-by-side baro/range altitude with online
vs offline correction applied.

**Online vs Offline Scatter** — Error vs thrust scatter for raw, online-
corrected, and offline-corrected data.

**CF Bandwidth Sensitivity** (range sensor required) — Sweeps `SENS_BAR_CF_BW`
to show how the CF crossover frequency affects K identification and
compensation quality. See [CF Bandwidth Tuning](#cf-bandwidth-tuning) for
how to interpret this page.

## Manual Calibration Procedure

If not using the online estimator, you can calibrate manually:

1. **Disable existing compensation**:
   ```
   param set SENS_BARO_PCOEF 0.0
   ```

2. **Fly a hover** at 2-5 m AGL for at least 60 seconds with some gentle
   altitude changes. A range sensor must be installed.

3. **Run this tool** on the log and apply the recommended parameter:
   ```
   param set SENS_BARO_PCOEF <value>
   ```

4. **Fly again** and re-run the tool to verify (it will auto-detect validation
   mode when compensation parameters are active).

## Interpreting Results

| Metric | Good | Marginal | Poor |
|--------|------|----------|------|
| Thrust correlation abs(r) | > 0.6 | 0.3 - 0.6 | < 0.3 |
| Model R^2 | > 0.3 | 0.1 - 0.3 | < 0.1 |
| Compensated abs(r) | < 0.2 | 0.2 - 0.4 | > 0.4 |

- **Low R^2**: Thrust is not the dominant baro error source. Consider thermal
  drift, ground effect, or sensor placement issues.
- **Very large K (> 5 m)**: May indicate a sensor mounting issue. The baro
  should be shielded from direct propwash where possible.
- **Online/offline K disagreement > 2 m**: The estimator may not have had
  enough excitation. Fly longer or with more altitude variation.

## CF Bandwidth Tuning

The default `SENS_BAR_CF_BW` (0.05 Hz) works well for most vehicles. Only
adjust it if the online estimator consistently fails to converge or produces
a K that disagrees with range-sensor ground truth.

When a range sensor is present, the tool generates a **CF Bandwidth
Sensitivity** page with two panels:

**Left — K vs Bandwidth**: Shows how the identified K changes with CF
crossover frequency. The green dashed line is the range-sensor ground truth K.

- If the curve is **flat near ground truth** around the default (gray line):
  the default bandwidth is fine, K identification is robust.
- If the curve **crosses ground truth far from the default**: the default
  bandwidth is producing a biased K. Consider setting `SENS_BAR_CF_BW` to
  the "Best K match" bandwidth (blue dashed line).

**Right — Compensated Error Std vs Bandwidth**: Shows the residual baro
error standard deviation after applying the K identified at each bandwidth.
Lower is better.

- The green line is the theoretical minimum (range-sensor optimal PCOEF).
- The orange line is the current PCOEF performance.
- If the default bandwidth is already near the minimum: leave it alone.
- If a different bandwidth gives significantly lower error std: consider
  changing `SENS_BAR_CF_BW` to that value.

In practice, the default is conservative and works across vehicle types.
Raising the bandwidth makes K identification faster and more accurate when
the IMU is good, but noisier when it's not. Lower bandwidth is more robust
to IMU vibration but slower to converge.

## Parameters Reference

| Parameter | Description | Range | Default |
|-----------|-------------|-------|---------|
| `SENS_BARO_PCOEF` | Baro altitude correction per unit vertical thrust [m] | -30 to 30 | 0.0 |
| `SENS_BAR_AUTOCAL` | Bitmask: bit 0 = GNSS offset, bit 1 = online thrust cal | 0 to 3 | 1 |
| `SENS_BAR_CF_BW` | CF crossover frequency for the online estimator [Hz] | 0.01 to 1.0 | 0.05 |
