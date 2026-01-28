# Level Horizon Calibration

You can use _Level Horizon Calibration_ to compensate for small misalignments in controller orientation and to level the horizon in the _QGroundControl_ flight view (blue on top and green on bottom).

:::tip
Performing this calibration step is only recommended if the autopilot's orientation is visibly misaligned with the specified orientation, or if there is a constant drift during flight in not position-controlled flight modes.
:::

## Performing the Calibration

To level the horizon:

1. Start _QGroundControl_ and connect the vehicle.
1. Select the **Gear** icon (Vehicle Setup) in the top toolbar and then **Sensors** in the sidebar.
1. Click the **Level Horizon** button.
   ![Level Horizon calibration](../../assets/qgc/setup/sensor/sensor_level_horizon.png)
   ::: info
   You should already have set the [Autopilot Orientation](../config/flight_controller_orientation.md). If not, you can also set it here.
   :::
1. Place the vehicle in its level flight orientation on a level surface:

   - For planes this is the position during level flight (planes tend to have their wings slightly pitched up!)
   - For copters this is the hover position.

1. Press **OK** to start the calibration process.
1. Wait until the calibration process is finished.

## Verification

Check that the artificial horizon displayed in the flight view has the indicator in the middle when the vehicle is placed on a level surface.

## Further Information

- [Advanced Orientation Tuning](../advanced_config/advanced_flight_controller_orientation_leveling.md) (advanced users only).
- [QGroundControl User Guide > Sensors](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html#level-horizon)
- [PX4 Setup Video "Gyroscope" - @1m14s](https://youtu.be/91VGmdSlbo4?t=1m14s) (Youtube)
