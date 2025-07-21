# IMU/Compass Factory Calibration

PX4 OEM manufacturers can perform an IMU and compass factory calibration in order to store values for the accelerometer, gyroscope and magnetometer calibration into persistent storage (usually EEPROM).
This ensures that end users can always reset vehicle configurations and tuning to a safe state for flying.

This procedure will write the following parameters to `/fs/mtd_caldata`: [CAL_ACC\*](../advanced_config/parameter_reference.md#CAL_ACC0_ID), [CAL_GYRO\*](../advanced_config/parameter_reference.md#CAL_GYRO0_ID), [CAL_MAG\*](../advanced_config/parameter_reference.md#CAL_MAG0_ID).
This data will then be used when the parameters are set (or reset) to their default values.

:::warning
This feature relies on the FMU having a dedicated EEPROM chip or an accompanying IMU PCBA that has sufficient space for the data.
PX4 will store the data to `/fs/mtd_caldata`, creating the file if necessary.
:::

::: info
These values cannot be stored in the [frame configuration](../dev_airframes/adding_a_new_frame.md) because they vary from device to device (the frame configuration defines the set of parameters that are applicable across all vehicles of the same type, such as the enabled sensors, [autopilot rotation](../config/flight_controller_orientation.md) and PID tuning).
:::

## Performing the Factory Calibration

1. Set the parameter [SYS_FAC_CAL_MODE](../advanced_config/parameter_reference.md#SYS_FAC_CAL_MODE) to 1.
1. Perform all IMU calibrations: [accelerometer](../config/accelerometer.md#performing-the-calibration), [gyroscope](../config/gyroscope.md#performing-the-calibration) and [magnetometer](../config/compass.md#performing-the-calibration).
1. Reboot the vehicle.
   This will write all `CAL_ACC*`, `CAL_GYRO*` and `CAL_MAG*` parameters into `/fs/mtd_caldata`.
1. Set the parameter `SYS_FAC_CAL_MODE` back to 0 (default).

::: info
If you only want to factory calibrate the accelerometer and the gyroscope you can set [SYS_FAC_CAL_MODE](../advanced_config/parameter_reference.md#SYS_FAC_CAL_MODE) to 2, in which case the magnetometer is omitted.
:::

Subsequent user calibrations will then take effect as usual (factory calibration data is only used for the parameter default values).

## Further Information

- [QGroundControl User Guide > Sensors](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html)
