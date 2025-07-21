# Gyroscope Calibration

_QGroundControl_ will guide you to place the vehicle on a flat surface and keep it still.

## 执行校准

The calibration steps are:

1. Start _QGroundControl_ and connect the vehicle.

2. Select **"Q" icon > Vehicle Setup > Sensors** (sidebar) to open _Sensor Setup_.

3. Click the **Gyroscope** sensor button.

   ![Select Gyroscope calibration PX4](../../assets/qgc/setup/sensor/gyroscope_calibrate_px4.png)

4. Place the vehicle on a surface and leave it still.

5. Click **Ok** to start the calibration.

   The bar at the top shows the progress:

   ![Gyro calibration in progress on PX4](../../assets/qgc/setup/sensor/gyroscope_calibrate_progress_px4.png)

6. When finished, _QGroundControl_ will display a progress bar _Calibration complete_
   ![Gyro calibration complete on PX4](../../assets/qgc/setup/sensor/gyroscope_calibrate_complete_px4.png)

:::info
If you move the vehicle _QGroundControl_ will automatically restart the gyroscope calibration.
:::

## 更多信息

- [QGroundControl User Guide > Gyroscope](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html#gyroscope)
