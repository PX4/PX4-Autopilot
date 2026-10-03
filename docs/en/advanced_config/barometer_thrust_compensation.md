# Barometer Thrust Compensation

Propellers change the static pressure at the barometer in proportion to motor output.
On small vehicles with the barometer close to the propellers the resulting altitude error reaches several metres and tracks every thrust change.

PX4 corrects the barometer altitude with a term proportional to the vertical thrust setpoint:

```txt
baro_alt_meter = raw_altitude + SENS_BARO_K_T * |thrust_z|
```

`thrust_z` is the Z body-axis component of `vehicle_thrust_setpoint`, so the correction is zero in fixed-wing flight.
The correction is applied in the sensors module and logged in `vehicle_air_data.baro_alt_correction`, together with the [static pressure](../advanced_config/static_pressure_buildup.md) correction.

## Calibration

1. Set [SENS_BARO_K_T](../advanced_config/parameter_reference.md#SENS_BARO_K_T) to 0.
2. Hover at 2-5 m for at least 60 seconds with gentle altitude changes.
   A downward-facing [distance sensor](../sensor/rangefinders.md) provides the ground truth; without one the script estimates the error from the accelerometer instead, which is less accurate.
3. Run the analysis on the log:

   ```sh
   python3 Tools/baro_compensation/baro_thrust_calibration.py <log.ulg>
   ```

4. Set `SENS_BARO_K_T` to the printed value and fly again to verify.

The sign depends on where the barometer sits in the propeller flow: negative if the barometer altitude rises with thrust, positive if it drops.

::: info
Thrust and airspeed both rise in forward flight, so their barometer errors are correlated.
Identify `SENS_BARO_K_T` from a hover log first, then tune the [SENS_BARO_K\_\*](../advanced_config/parameter_reference.md#SENS_BARO_K_XP) airspeed coefficients with it applied.
:::

::: info
One coefficient applies to whichever barometer is selected.
Barometers mounted at different positions, for example one on the flight controller and one on a CAN node, see different propeller flow and cannot both be compensated.
:::

## See Also

- [Static Pressure Buildup](../advanced_config/static_pressure_buildup.md)
- [Compass Power Compensation](../advanced_config/compass_power_compensation.md)
