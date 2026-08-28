# Compass Power Compensation

Compensate magnetometer readings for magnetic fields from motor current when the compass cannot be moved away from the wiring. Cables must be fixed in place; moving them invalidates the coefficients.

## Data collection

1. [Compass calibration](../config/compass.md#compass-calibration).
2. [CAL_MAG_COMP_TYP](../advanced_config/parameter_reference.md#CAL_MAG_COMP_TYP) = 0.
3. Enable [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE) bit 11 (High rate sensors).
4. Fly a couple of minutes with throttle changes. A restrained throttle ramp with props on also works.

## Identification

```sh
python src/modules/sensors/vehicle_magnetometer/mag_compensation/python/mag_compensation.py LOG.ulg
```

Optional second argument: `current` (default if battery current is in the log) or `thrust`. `--instance N` selects the battery/thrust instance.

The script prints `CAL_MAG_COMP_TYP` and `CAL_MAGx_{X,Y,Z}COMP`. It estimates a bulk delay from the field norm vs current, then fits per-axis coefficients after removing the Earth field using `vehicle_attitude`. Current compensation is preferred when battery current is logged.

Set the printed parameters. [CAL_MAG_COMP_TYP](../advanced_config/parameter_reference.md#CAL_MAG_COMP_TYP) is 1 for thrust, 2 or 3 for battery current instance 0 or 1.

## See Also

- [OEM/Factory Configuration](../advanced_config/oem.md)
