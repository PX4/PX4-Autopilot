# Airspeed Calibration

::: info
[Airspeed sensors](../sensor/airspeed.md) are highly recommended for Fixed-wing and VTOL vehicles.
:::

:::warning
Unlike most other sensor drivers, the airspeed sensor drivers are not automatically started.
Before calibration they must be [enabled via the corresponding parameter](../advanced_config/parameters.md):

- Sensirion SDP3X ([SENS_EN_SDP3X](../advanced_config/parameter_reference.md#SENS_EN_SDP3X))
- TE MS4525 ([SENS_EN_MS4525DO](../advanced_config/parameter_reference.md#SENS_EN_MS4525DO))
- TE MS5525 ([SENS_EN_MS5525DS](../advanced_config/parameter_reference.md#SENS_EN_MS5525DS))
- Eagle Tree airspeed sensor ([SENS_EN_ETSASPD](../advanced_config/parameter_reference.md#SENS_EN_ETSASPD))
  :::

## Performing the Calibration

To calibrate the airspeed sensor:

1. Start _QGroundControl_ and connect the vehicle.
1. Enable the airspeed sensors if not already done (as in _warning_ above).
1. Select **"Q" icon > Vehicle Setup > Sensors** (sidebar) to open _Sensor Setup_.
1. Click the **Airspeed** sensor button.

   ![Airspeed calibration](../../assets/qgc/setup/sensor/sensor_airspeed.jpg)

1. Shield the sensor from the wind (i.e. cup it with your hand).
   Take care not to block any of its holes.
1. Click **OK** to start the calibration.
1. Once asked for, blow into the tip of the pitot tube to signal the end of calibration.

   :::tip
   Blowing into the tube is also a basic check that the dynamic and static ports are installed correctly.
   If they are swapped then the sensor will read a large negative differential pressure when you blow into the tube, and the calibration will abort with an error.
   :::

1. _QGroundControl_ then tells you if the calibration was successful or not.

## Further Information

- [QGroundControl User Guide > Sensors](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html#airspeed)
