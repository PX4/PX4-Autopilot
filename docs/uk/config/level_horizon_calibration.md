# Калібрування рівня горизонту

You can use _Level Horizon Calibration_ to compensate for small misalignments in controller orientation and to level the horizon in the _QGroundControl_ flight view (blue on top and green on bottom).

:::tip
Performing this calibration step is only recommended if the autopilot's orientation is visibly misaligned with the specified orientation, or if there is a constant drift during flight in not position-controlled flight modes.
:::

## Виконання калібрування

Вирівняти горизонт:

1. Start _QGroundControl_ and connect the vehicle.

2. Select the **Gear** icon (Vehicle Setup) in the top toolbar and then **Sensors** in the sidebar.

3. Click the **Level Horizon** button.
   ![Level Horizon calibration](../../assets/qgc/setup/sensor/sensor_level_horizon.png)
   ::: info
   You should already have set the [Autopilot Orientation](../config/flight_controller_orientation.md). Якщо ні, ви також можете встановити це тут.

:::

4. Помістіть транспортний засіб на рівну відстань на рівній поверхні:

   - Для літаків це положення під час рівнопланового польоту (літаки мають тенденцію трохи підняти свої крила!)
   - Для коптерів це позиція утримання.

5. Press **OK** to start the calibration process.

6. Зачекайте, доки завершиться процес калібрування.

## Перевірка

Перевірте, що штучний горизонт, відображений у виді польоту, має показник посередині, коли транспортний засіб розміщений на рівній поверхні.

## Подальша інформація

- [Advanced Orientation Tuning](../advanced_config/advanced_flight_controller_orientation_leveling.md) (advanced users only).
- [QGroundControl User Guide > Sensors](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html#level-horizon)
- [PX4 Setup Video "Gyroscope" - @1m14s](https://youtu.be/91VGmdSlbo4?t=1m14s) (Youtube)
