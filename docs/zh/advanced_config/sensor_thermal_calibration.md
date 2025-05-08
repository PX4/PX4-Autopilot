# 热校准和补偿

PX4 contains functionality to calibrate and compensate accelerometer, gyro, magnetometer, and barometric pressure sensors for the effect of changing sensor temperature on sensor bias.

This topic details the [test environment](#test_setup) and [calibration procedures](#calibration_procedures).
At the end there is a description of the [implementation](#implementation).

:::info
After thermal calibration, the thermal calibration parameters (`TC_*`) are used for _all_ calibration/compensation of the respective sensors.
Any subsequent standard calibration will therefore update `TC_*` parameters and not the "normal" `SYS_CAL_*` calibration parameters (and in some cases these parameters may be reset).
:::

:::info
Releases up to PX4 v1.14, do not support thermal calibration of the magnetometer.
:::

<a id="test_setup"></a>

## 测试设置/最佳实践

The [calibration procedures](#calibration_procedures) described in the following sections are ideally run in an _environmental chamber_ (a temperature and humidity controlled environment) as the board is heated from the lowest to the highest operating/calibration temperature.
Before starting the calibration, the board is first _cold soaked_ (cooled to the minimum temperature and allowed to reach equilibrium).

:::info
Active electric heating elements will affect the magnetometer calibration values.
Ensure that heating elements are either inactive or sufficiently far from the sensor to avoid injecting noise into the magnetometer calibration.
:::

对于冷却，您可以使用普通的家用冰箱达到 -20C，商用冰箱可以达到 -40C 的量级。
The board should be placed in a ziplock/anti-static bag containing a silica packet, with a power lead coming out through a sealed hole.
冷却后，可将袋子移至测试环境，并在同一袋中继续测试。

:::info
The bag/silica is to prevent condensation from forming on the board.
:::

它可以在没有商业级环境房间的情况下进行校准。
可以使用内部空间很小的泡沫塑料盒来创造一个简单的环境容器。
这允许自驾仪将空气相对快速地自加热（确保盒子有一个小孔以平衡容器内外压力，但仍然能够在容器内加热）。

使用这种设置可以将电路板加热到约 70C 。
经验表明，许多普通电路板可以加热到这个温度不会产生不良副作用。
如有疑问，请与制造商核实安全操作的温度范围。

:::tip
To check the status of the onboard thermal calibration use the MAVlink console (or NuttX console) to check the reported internal temp from the sensor.
:::

<a id="calibration_procedures"></a>

## 校准过程

PX4 支持两种校准过程：

- [onboard](#onboard_calibration) - calibration is run on the board itself. 该方法需要知道测试设置中可实现的温升量。
- [offboard](#offboard_calibration) - compensation parameters are calculated on a development computer based on log information collected during the calibration procedure. 该方法允许用户可视地检查数据和曲线拟合的质量。

The offboard approach is more complex and slower, but requires less knowledge of the test setup and is easier to validate.

<a id="onboard_calibration"></a>

### 板载校准过程

Onboard calibration is run entirely on the device. It require knowledge of the amount of temperature rise that is achievable with the test setup.

To perform and onboard calibration:

1. 确保在校准前设置机架类型，否则在设置飞控板时校准参数将丢失。
2. Power the board and set the `SYS_CAL_*` parameters to 1 to enable calibration of the required sensors at the next startup. [^1]
3. Set the [SYS_CAL_TDEL](../advanced_config/parameter_reference.md#SYS_CAL_TDEL) parameter to the number of degrees of temperature rise required for the onboard calibrator to complete. 如果此参数太小，则校准将提前完成，并且校准的温度范围将不足以在电路板完全预热时进行补偿。 如果此参数设置得太大，则板载校准器将永远不会完成。 在设置此参数时，应考虑到电路板自加热导致的温度升高。 如果传感器的温升量未知，则应使用板外校准方法。
4. Set the [SYS_CAL_TMIN](../advanced_config/parameter_reference.md#SYS_CAL_TMIN) parameter to the lowest temperature data that you want the calibrator to use. 更低的冷却温度能够用于减少冷却时间，同时保持对校准最低温度的控制。 如果校准器温度低于此参数设置的值，则不会使用传感器的数据。
5. Set the [SYS_CAL_TMAX](../advanced_config/parameter_reference.md#SYS_CAL_TMAX) parameter to the highest starting sensor temperature that should be accepted by the calibrator. 如果起始温度高于此参数设置的值，校准将退出并报告错误。 Note that if the variation in measured temperature between different sensors exceeds the gap between `SYS_CAL_TMAX` and `SYS_CAL_TMIN`, then it will be impossible for the calibration to start.
6. Remove power and cold soak the board to below the starting temperature specified by the `SYS_CAL_TMIN` parameter. 请注意，在校准开始之前启动过程有10秒的延迟，以允许所有传感器稳定，并且传感器在此期间会内部发热。
7. Keeping the board stationary[^2], apply power and warm to a temperature high enough to achieve the temperature rise specified by the `SYS_CAL_TDEL` parameter. 校准期间，完成百分比将打印到系统控制台。 [^3]
8. 校准完成后，断开电源，让电路板冷却到校准范围内的温度，然后再执行下一步。
9. Perform a 6-point accel calibration via the system console using `commander calibrate accel` or via _QGroundControl_. 如果首次设置电路板，则还需要执行陀螺仪和磁力计校准。
10. 在任何传感器校准之后的首次飞行之前，电路板必须重新上电，因为校准带来的突然的偏移变化可能会扰乱导航估计器，并且某些参数直到下次启动时才会被使用它们的算法加载。

<a id="offboard_calibration"></a>

### 板外校准过程

Offboard calibration is run on a development computer using data collected during the calibration test. This method provides a way to visually check the quality of data and curve fit.

To perform an offboard calibration:

1. 确保在校准前设置机架类型，否则在设置飞控板时校准参数将丢失。

2. Power up the board and set the [TC_A_ENABLE](../advanced_config/parameter_reference.md#TC_A_ENABLE), [TC_B_ENABLE](../advanced_config/parameter_reference.md#TC_B_ENABLE), [TC_G_ENABLE](../advanced_config/parameter_reference.md#TC_G_ENABLE), and [TC_M_ENABLE](../advanced_config/parameter_reference.md#TC_M_ENABLE) parameters to `1`.

3. Set all [CAL_ACC\*](../advanced_config/parameter_reference.md#CAL_ACC0_ID), [CAL_GYRO\*](../advanced_config/parameter_reference.md#CAL_GYRO0_ID), [CAL_MAG\*](../advanced_config/parameter_reference.md#CAL_MAG0_ID), and [CAL_BARO\*](../advanced_config/parameter_reference.md#CAL_BARO0_ID) parameters to defaults.

4. Set the [SDLOG_MODE](../advanced_config/parameter_reference.md#SDLOG_MODE) parameter to 2 to enable logging of data from boot.

5. Set the [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE) checkbox for _thermal calibration_ (bit 2) to log the raw sensor data required for calibration.

6. 将电路板冷却到操作所需的最低温度。

7. Apply power and keeping the board still [^2], warm it slowly to the maximum required operating temperature. [^3]

8. 断开电源并取出 .ulog 文件。

9. Open a terminal window in the **Firmware/Tools** directory and run the python calibration script:

  ```sh
  python process_sensor_caldata.py <full path name to .ulog file>
  ```

  This will generate a **.pdf** file showing the measured data and curve fits for each sensor, and a **.params** file containing the calibration parameters.

10. Power the board, connect _QGroundControl_ and load the parameter from the generated **.params** file onto the board using _QGroundControl_. 由于参数的数量，加载它们可能需要一些时间。

11. After parameters have finished loading, set `SDLOG_MODE` to 1 to re-enable normal logging and remove power.

12. Power the board and perform a normal accelerometer sensor calibration using _QGroundControl_. 重要的是，此步骤在飞控板处于校准温度范围内进行。 此步骤后的首次飞行之前，应重新启动电路板，因为突然的偏置变化会扰乱导航估计器，并且某些参数直到下次启动时才会被使用它们的算法加载。

<a id="implementation"></a>

## 实施细节

Calibration refers to the process of measuring the change in sensor value across a range of internal temperatures, and performing a polynomial fit on the data to calculate a set of coefficients (stored as parameters) that can be used to correct the sensor data. Compensation refers to the process of using the internal temperature to calculate an offset that is subtracted from the sensor reading to correct for changing offset with temperature

The accelerometer, gyro, and magnetometer sensor offsets are calculated using a 3rd order polynomial, whereas the barometric pressure sensor offset is calculated using a 5th order polynomial. Example fits are show below:

![Thermal calibration accel](../../assets/calibration/thermal_calibration_accel.png)

![Thermal calibration gyro](../../assets/calibration/thermal_calibration_gyro.png)

![Thermal calibration mag](../../assets/calibration/thermal_calibration_mag.png)

![Thermal calibration barometer](../../assets/calibration/thermal_calibration_baro.png)

### 校准参数存储

With the existing parameter system implementation we are limited to storing each value in the struct as a separate entry. To work around this limitation the following logical naming convention is used for the [thermal compensation parameters](../advanced_config/parameter_reference.md#thermal-compensation):

```sh
TC_[type][instance]_[cal_name]_[axis]
```

Where:

- `type`: is a single character indicating the type of sensor where `A` = accelerometer, `G` = rate gyroscope, `M` = magnetometer, and `B` = barometer.

- `instance`: is an integer 0,1 or 2 allowing for calibration of up to three sensors of the same `type`.

- `cal_name`: is a string identifying the calibration value. 它具有可能的值如下：

  - `Xn`: Polynomial coefficient where n is the order of the coefficient, e.g. `X3 * (temperature - reference temperature)**3`.
  - `SCL`: scale factor.
  - `TREF`: reference temperature (deg C).
  - `TMIN`: minimum valid temperature (deg C).
  - `TMAX`: maximum valid temperature (deg C).

- `axis`: is an integer 0,1 or 2 indicating that the calibration data is for X,Y or Z axis in the board frame of reference. For the barometric pressure sensor, the `axis` suffix is omitted.

Examples:

- [TC_A1_TREF](../advanced_config/parameter_reference.md#TC_A1_TREF) is the reference temperature for the second accelerometer.
- [TC_G0_X3_0](../advanced_config/parameter_reference.md#TC_G0_X3_0) is the `^3` coefficient for the first gyro x-axis.

### 校准参数使用

The correction for thermal offsets (using the calibration parameters) is performed in the [sensors module](../modules/modules_system.md#sensors).
The reference temperature is subtracted from the measured temperature to obtain a delta temperature where:

```
delta = measured_temperature - reference_temperature
```

The delta temperature is then used to calculate a offset, where:

```
offset = X0 + X1*delta + X2*delta**2 + ... + Xn*delta**n
```

The offset and temperature scale factor are then used to correct the sensor measurement where:

```
corrected_measurement = (raw_measurement - offset) * scale_factor
```

If the temperature is above the test range set by the `*_TMIN` and `*_TMAX` parameters, then the measured temperature will be clipped to remain within the limits.

Correction of the accelerometer, gyroscope, magnetometer, or barometer data is enabled by setting [TC_A_ENABLE](../advanced_config/parameter_reference.md#TC_A_ENABLE), [TC_G_ENABLE](../advanced_config/parameter_reference.md#TC_G_ENABLE), [TC_M_ENABLE](../advanced_config/parameter_reference.md#TC_M_ENABLE), or [TC_B_ENABLE](../advanced_config/parameter_reference.md#TC_B_ENABLE) parameters to 1 respectively.

### Compatibility with legacy `CAL_*` parameters and commander controlled calibration

The legacy temperature-agnostic PX4 rate gyro and accelerometer sensor calibration is performed by the commander module and involves adjusting offset, and in the case of accelerometer calibration, scale factor calibration parameters. The offset and scale factor parameters are applied within the driver for each sensor. These parameters are found in the [CAL parameter group](../advanced_config/parameter_reference.md#sensor-calibration).

Onboard temperature calibration is controlled by the events module and the corrections are applied within the sensors module before the sensor combined uORB topic is published. This means that if thermal compensation is being used, all of the corresponding legacy offset and scale factor parameters must be set to defaults of zero and unity before a thermal calibration is performed. If an on-board temperature calibration is performed, this will be done automatically, however if an offboard calibration is being performed it is important that the legacy `CAL*OFF` and `CAL*SCALE` parameters be reset before calibration data is logged.

If accel thermal compensation has been enabled by setting the `TC_A_ENABLE` parameter to 1, then the commander controlled 6-point accel calibration can still be performed.
However, instead of adjusting the `*OFF` and `*SCALE` parameters in the `CAL` parameter group, these parameters are set to defaults and the thermal compensation `X0` and `SCL` parameters are adjusted instead.

If gyro thermal compensation has been enabled by setting the `TC_G_ENABLE` parameter to 1, then the commander controlled gyro calibration can still be performed, however it will be used to shift the compensation curve up or down by the amount required to zero the angular rate offset. It achieves this by adjusting the X0 coefficients.

If magnetometer thermal compensation has been enabled by setting the `TC_M_ENABLE` parameter to 1, then the commander controlled 6-point accel calibration can still be performed.
However, instead of adjusting the `*OFF` and `*SCALE` parameters in the `CAL` parameter group, these parameters are set to defaults and the thermal compensation `X0` and `SCL` parameters are adjusted instead.

### 局限

Scale factors are assumed to be temperature invariant due to the difficulty associated with measuring these at different temperatures. This limits the usefulness of the accelerometer calibration to those sensor models with stable scale factors. In theory with a thermal chamber or IMU heater capable of controlling IMU internal temperature to within a degree, it would be possible to perform a series of 6 sided accelerometer calibrations and correct the accelerometers for both offset and scale factor. Due to the complexity of integrating the required board movement with the calibration algorithm, this capability has not been included.

---

[^1]: The [SYS_CAL_ACCEL](../advanced_config/parameter_reference.md#SYS_CAL_ACCEL), [SYS_CAL_BARO](../advanced_config/parameter_reference.md#SYS_CAL_BARO) and [SYS_CAL_GYRO](../advanced_config/parameter_reference.md#SYS_CAL_GYRO) parameters are reset to 0 when the calibration is started.

[^2]: 气压传感器偏置的校准需要一个稳定的气压环境。 The air pressure will change slowly due to weather and inside buildings can change rapidly due to external wind fluctuations and HVAC system operation.

[^3]: Care must be taken when warming a cold soaked board to avoid formation of condensation on the board that can cause board damage under some circumstances.
