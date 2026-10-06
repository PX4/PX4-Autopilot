# Barometer compensation tuning

Two scripts identify the `SENS_BARO_K_*` coefficients that `VehicleAirData` applies to the barometer altitude before it is published.

| Script | Identifies | Reference | Flight profile |
|---|---|---|---|
| `baro_thrust_calibration.py` | `SENS_BARO_K_T` (propwash, linear in `|thrust_z|`) | `distance_sensor` if logged, else a baro/accel complementary filter | Hover at 2-5 m AGL for 60 s or more with gentle climbs and descents, little horizontal motion |
| `baro_static_pressure_compensation_tuning.py` | `SENS_BARO_K_XP/XN/YP/YN/Z` (airflow, quadratic in airspeed) | GNSS altitude | Position mode, forwards/backwards/left/right/up/down between rest and maximum speed, still air |

```sh
pip install -r requirements.txt
python3 baro_thrust_calibration.py <log.ulg>            # writes <log>.pdf next to the log
python3 baro_static_pressure_compensation_tuning.py <log.ulg>
```

Each script prints `param set ...` lines for the identified values. `baro_thrust_calibration.py` takes `--method range|cf_rls` to force the reference and `--output-dir` for the report.

## Log contents

| Topic | Thrust | Static pressure |
|---|---|---|
| `vehicle_air_data` | required | required |
| `vehicle_thrust_setpoint` | required | |
| `vehicle_status`, `vehicle_land_detected`, `vehicle_local_position` | gating | required |
| `vehicle_acceleration`, `vehicle_attitude` | `cf_rls` method | attitude required |
| `distance_sensor` | `range` method | |
| `vehicle_gnss` | | required |

`vehicle_air_data.baro_alt_meter` is logged after compensation. Both scripts subtract `baro_alt_correction` when the log has it so the fit yields the full coefficient; on older logs the thrust script undoes `SENS_BARO_K_T` from the logged parameter instead, and static pressure terms stay in the data.

## Order

Forward flight raises thrust and dynamic pressure together, so a fit over such a log attributes some of each error to the other term. Identify `SENS_BARO_K_T` from a hover log first, apply it, then fly the static pressure profile and identify `SENS_BARO_K_*` with the thrust term already active.
