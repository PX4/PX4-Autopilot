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
corrected_baro_alt = raw_baro_alt + SENS_BARO_PCOEF * filtered_thrust
```

where `filtered_thrust` is the normalized thrust magnitude [0, 1] passed through
a first-order low-pass filter with time constant `SENS_BARO_PTAU`. The filter
models the delay between a thrust setpoint change and the resulting pressure
change at the sensor (motor spin-up, propwash development, sensor response).

## How Calibration Works

### Online (Preferred)

Enable the online estimator by setting `SENS_BAR_AUTOCAL` bit 1 (value 2 or 3).
The `baro_thrust_estimator` module identifies `SENS_BARO_PCOEF` and
`SENS_BARO_PTAU` automatically during flight using an accel-baro complementary
filter and parallel RLS estimation. Parameters are saved to flash on disarm once
converged. No range sensor required.

### Offline (This Tool)

This tool identifies the same parameters from a flight log using a range sensor
as ground truth. Useful for:
- Initial calibration before enabling the online estimator
- Validating online estimator results against ground truth
- Analyzing compensation effectiveness

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
- Tau bank selection over time
- Prediction error and thrust excitation
- Convergence status timeline
- CF residual vs thrust scatter

### 2. Full Validation (online estimator + range sensor)

Cross-validates the online estimator against range-sensor ground truth:
- All estimator review plots (above)
- Range-based error analysis with before/after compensation
- Side-by-side comparison of online vs offline parameter identification

### 3. Standalone Calibration (range sensor, no estimator)

Identifies parameters from baro vs range error (original workflow):
- Cross-correlation delay analysis
- Grid search over filter time constants
- Recommended PCOEF and PTAU values

## Usage

```bash
python3 baro_thrust_calibration.py <path/to/log.ulg> [--output-dir <dir>]
```

## Output

### Console

Summary including parameters found in the log, online estimator convergence
status, range-based error statistics, and cross-validation between online and
offline identification (when both are available).

### PDF Report (`baro_calibration.pdf`)

Pages vary by mode. When the online estimator is present:

**Estimator Convergence** — K estimate with variance band, tau selection,
error variance and thrust excitation, convergence flags over time.

**Residual Analysis** — CF residual time series and residual-vs-thrust
scatter showing the linear relationship the estimator is fitting.

When a range sensor is present, additional pages show:

**Data Overview** — Baro vs range altitude, baro error, and thrust over time.

**Calibration/Validation** — Cross-correlation, R^2 vs tau sweep,
before/after compensation, recommended or validated parameters.

**Scatter** — Error vs thrust before and after compensation.

## Manual Calibration Procedure

If not using the online estimator, you can calibrate manually:

1. **Disable existing compensation**:
   ```
   param set SENS_BARO_PCOEF 0.0
   param set SENS_BARO_PTAU 0.0
   ```

2. **Fly a hover** at 2-5 m AGL for at least 60 seconds with some gentle
   altitude changes and throttle variation. A range sensor must be installed.

3. **Run this tool** on the log and apply the recommended parameters:
   ```
   param set SENS_BARO_PCOEF <value>
   param set SENS_BARO_PTAU <value>
   ```

4. **Fly again** and re-run the tool to verify (it will auto-detect validation
   mode when compensation parameters are active).

## Interpreting Results

| Metric | Good | Marginal | Poor |
|--------|------|----------|------|
| Thrust correlation |r| | > 0.6 | 0.3 - 0.6 | < 0.3 |
| Model R^2 | > 0.3 | 0.1 - 0.3 | < 0.1 |
| Compensated |r| | < 0.2 | 0.2 - 0.4 | > 0.4 |

- **Low R^2**: Thrust is not the dominant baro error source. Consider thermal
  drift, ground effect, or sensor placement issues.
- **Very large K (> 5 m)**: May indicate a sensor mounting issue. The baro
  should be shielded from direct propwash where possible.
- **Online/offline K disagreement > 1 m**: The estimator may not have had
  enough excitation. Fly longer or with more altitude variation.

## Parameters Reference

| Parameter | Description | Range | Default |
|-----------|-------------|-------|---------|
| `SENS_BARO_PCOEF` | Baro altitude correction per unit thrust [m] | -30 to 30 | 0.0 |
| `SENS_BARO_PTAU` | Thrust filter time constant [s] | 0 to 2 | 0.0 |
| `SENS_BAR_AUTOCAL` | Bitmask: bit 0 = GNSS offset, bit 1 = online thrust cal | 0 to 3 | 1 |
