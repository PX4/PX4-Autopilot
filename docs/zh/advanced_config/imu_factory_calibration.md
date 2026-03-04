# IMU/指南针工厂校准

PX4 OEM制造商可以执行IMU和罗盘工厂校准，以便将加速度计、陀螺仪和磁力计校准的数值存储到持久存储器中（通常是EEPROM）。
这将确保最终用户总是能够重置车辆配置并调整到安全状态以供飞行。

This procedure will write the following parameters to `/fs/mtd_caldata`: [CAL_ACC\*](../advanced_config/parameter_reference.md#CAL_ACC0_ID), [CAL_GYRO\*](../advanced_config/parameter_reference.md#CAL_GYRO0_ID), [CAL_MAG\*](../advanced_config/parameter_reference.md#CAL_MAG0_ID).
当参数被设置（或重置）为其默认值时，此数据将被使用。

:::warning
This feature relies on the FMU having a dedicated EEPROM chip or an accompanying IMU PCBA that has sufficient space for the data.
PX4 will store the data to `/fs/mtd_caldata`, creating the file if necessary.
:::

:::info
These values cannot be stored in the [frame configuration](../dev_airframes/adding_a_new_frame.md) because they vary from device to device (the frame configuration defines the set of parameters that are applicable across all vehicles of the same type, such as the enabled sensors, [autopilot rotation](../config/flight_controller_orientation.md) and PID tuning).
:::

## 执行工厂校准

1. Set the parameter [SYS_FAC_CAL_MODE](../advanced_config/parameter_reference.md#SYS_FAC_CAL_MODE) to 1.
2. Perform all IMU calibrations: [accelerometer](../config/accelerometer.md#performing-the-calibration), [gyroscope](../config/gyroscope.md#performing-the-calibration) and [magnetometer](../config/compass.md#performing-the-calibration).
3. Reboot the vehicle.
   This will write all `CAL_ACC*`, `CAL_GYRO*` and `CAL_MAG*` parameters into `/fs/mtd_caldata`.
4. Set the parameter `SYS_FAC_CAL_MODE` back to 0 (default).

:::info
If you only want to factory calibrate the accelerometer and the gyroscope you can set [SYS_FAC_CAL_MODE](../advanced_config/parameter_reference.md#SYS_FAC_CAL_MODE) to 2, in which case the magnetometer is omitted.
:::

随后的用户校准将像往常一样生效（工厂校准数据仅用于参数默认值）。

## 更多信息

- [QGroundControl User Guide > Sensors](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html)
