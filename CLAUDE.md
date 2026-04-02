# PX4-Autopilot

## Build

```bash
make ark_fmu-v6x_default -j$(nproc)    # ARK FPV target
make px4_sitl_default -j$(nproc)        # SITL
```

## Conventions

- Commits: `type(scope): description` (conventional commits). No `Co-Authored-By`.
- PR titles: same conventional commit format. No Claude attribution.
- Push to `jake` remote for fork PRs, `origin` for upstream PRs.

## Active branch: `dakejahl/baro-thrust-estimator`

PR: https://github.com/dakejahl/PX4-Autopilot/pull/41

### What it does

Propwash-induced barometer error compensation. Propellers create static pressure changes at the baro sensor proportional to motor output (~7m error, r=0.91 on ARK FPV). Correction: `baro_alt += SENS_BARO_PCOEF * mean_motor_output`.

### Components

**Thrust compensation** (`src/modules/sensors/vehicle_air_data/VehicleAirData.cpp`)
- Motor ring buffer (8 samples) with time-matched lookup against baro `timestamp_sample`
- Param: `SENS_BARO_PCOEF` (m per unit motor output, identified as ~-20 to -22 on test vehicle)

**BMP388 timestamp fix** (`src/drivers/barometer/bmp388/bmp388.cpp`)
- `timestamp_sample` corrected from read time to integration midpoint (`- measurement_time/2`)
- Eliminated 38ms baro-thrust lag. Other baro drivers not yet corrected.

**Online estimator** (`src/modules/baro_thrust_estimator/`)
- CF (complementary filter) isolates thrust-correlated baro error at 0.05Hz crossover
- RLS (recursive least squares) identifies gain K online, saves to `SENS_BARO_PCOEF` on disarm
- Controlled by `SENS_BAR_AUTOCAL` bit 1

**Rate limiting refactor**
- Removed `SENS_BARO_RATE`/`SENS_MAG_RATE` from publishers (had aliasing bug)
- Added `EKF2_BARO_RATE`/`EKF2_MAG_RATE` in EKF2.cpp
- Param translations in `src/lib/parameters/param_translation.cpp`

**Analysis tools** (`Tools/baro_compensation/`)
- `baro_thrust_calibration.py <log.ulg>` — offline calibration/validation, PDF output
- `motor_latency_analysis.py <log.ulg>` — command-to-RPM delay analysis

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SENS_BARO_PCOEF` | 0.0 | Baro alt correction per unit motor output [m] |
| `SENS_BAR_AUTOCAL` | 3 | Bitmask: bit 0 = GNSS cal, bit 1 = thrust comp |
| `EKF2_BARO_RATE` | 20.0 | Max baro fusion rate in EKF [Hz] |
| `SENS_BAR_CF_BW` | 0.05 | CF crossover frequency for thrust estimator [Hz] |
| `EKF2_MAG_RATE` | 15.0 | Max mag fusion rate in EKF [Hz] |

### Test vehicle

ARK FPV: BMP390 baro (23Hz, 16x oversampling, 43ms forced-mode), 1404 4000KV motors, 3" triblade props, AM32 ESCs with BDShot, ARK Flow (range + optical flow) for validation.

### Key findings

- K ≈ 20-22m consistent across flights, online/offline agree within 0.5m
- Compensation: thrust r from 0.91 to 0.16, error std from 2.3m to 0.95m
- BMP390 integration window (37ms) smooths over motor response (~15ms), no delay param needed
- AM32 ESC RPM telemetry has ~6ms filtering latency (6-sample moving avg + IIR)

### Motor delay analysis (ARK FPV: 1404 4000KV + 3" triblade + AM32)

Measured 25ms command→RPM cross-correlation lag. Breakdown:
- ~15ms true electromechanical response (ESC commutation + prop inertia)
- ~6ms AM32 RPM filtering (6-sample commutation interval moving avg + IIR with alpha=0.25, see ~/code/ark/AM32/Src/main.c:1901,1626)
- ~4ms BDShot telemetry round-robin (4 motors at 800Hz output rate = 200Hz per channel)

The motor delay is invisible to baro-thrust compensation because the BMP390's 37ms integration window low-passes over it. A `SENS_BARO_PTAU` delay param was prototyped and removed for this reason. On faster baro sensors (lower oversampling, shorter integration) the motor delay could resurface.

### Next steps

- Install ARK Flow for position hold
- Test on other vehicles/baro sensors
- Correct `timestamp_sample` in other baro drivers (MS5611, DPS310, BMP280)
