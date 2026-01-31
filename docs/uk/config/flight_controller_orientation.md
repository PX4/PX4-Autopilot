# Орієнтація контролера польоту/сенсора

За замовчуванням плату управління польотом та зовнішні компаси (якщо вони є) повинні бути розміщені на верхній стороні рами, орієнтовані так, щоб стрілка вказувала вперед транспортного засобу.
Якщо плата або будь-які зовнішні компаси встановлені в іншому положенні, вам потрібно буде налаштувати це в програмному забезпеченні.

## Обчислення Орієнтації

Зміщення ROLL, PITCH та/або YAW контролера польоту обчислюються відносно транспортного засобу навколо передньої (x), правої (y), внизу (z) вісей.

![Frame Heading](../../assets/concepts/frame_heading.png)

Вісі, навколо яких обертається, залишаються тими самими з одного кроку обертання до наступного.
Таким чином, рамка для виконання обертання залишається нерухомою.
This is also known as _extrinsic rotation_.

![Vehicle orientation](../../assets/qgc/setup/sensor/fc_orientation_1.png)

For example, the vehicles shown below have rotations around the z-axis (i.e. yaw only) corresponding to: `ROTATION_NONE`, `ROTATION_YAW_90`,`ROTATION_YAW_180`,`ROTATION_YAW_270`.

![Yaw Rotation](../../assets/qgc/setup/sensor/yaw_rotation.png)

:::info
For a VTOL Tailsitter airframe set the vehicle orientation according to its multirotor configuration (i.e. relative to the vehicle during, takeoff, hovering, landing) for all sensor calibrations.

Вісь зазвичай відноситься до орієнтації транспортного засобу під час стабільного прямого польоту.
For more information see [Basic Concepts](../getting_started/px4_basic_concepts.md#heading-and-directions).
:::

## Налаштування орієнтації контролера польоту

Щоб встановити орієнтацію:

1. Start _QGroundControl_ and connect the vehicle.

2. Select **"Q" icon > Vehicle Setup > Sensors** (sidebar) to open _Sensor Setup_.

3. Select the **Orientations** button.

   ![Set sensor orientations](../../assets/qgc/setup/sensor/sensor_orientation_set_orientations.jpg)

4. Select the **AutoPilot Orientation** (as [calculated above](#calculating-orientation)).

   ![Orientation options](../../assets/qgc/setup/sensor/sensor_orientation_selector_values.jpg)

5. Press **OK**.

:::info
You can use [Level Horizon Calibration](../config/level_horizon_calibration.md) to compensate for small miss-alignments in controller orientation and to level the horizon in flight view.
:::

## Встановлення Орієнтації компаса

PX4 will automatically detect the compass orientation as part of [compass calibration](../config/compass.md) ([by default](../advanced_config/parameter_reference.md#SENS_MAG_AUTOROT)) for any of the [standard MAVLink orientations](https://mavlink.io/en/messages/common.html#MAV_SENSOR_ORIENTATION) (upright and facing forward, or any multiple of 45° offset in any axis) .

:::info
You can confirm that auto detection worked by looking at the [CAL_MAGn_ROT](../advanced_config/parameter_reference.md#CAL_MAG0_ROT) parameters.
:::

If a non-standard orientation has been used you will need to set the [CAL_MAGx_ROLL](../advanced_config/parameter_reference.md#CAL_MAG0_ROLL), [CAL_MAGx_PITCH](../advanced_config/parameter_reference.md#CAL_MAG0_PITCH), and [CAL_MAGx_YAW](../advanced_config/parameter_reference.md#CAL_MAG0_YAW) parameters for each compass to the angles that were used.

This will automatically set [CAL_MAGn_ROT](../advanced_config/parameter_reference.md#CAL_MAG0_ROT) to "custom euler angle" and prevents automatic calibration for the selected compass (even if [SENS_MAG_AUTOROT](../advanced_config/parameter_reference.md#SENS_MAG_AUTOROT) is set).

## Подальша інформація

- [Advanced Orientation Tuning](../advanced_config/advanced_flight_controller_orientation_leveling.md) (advanced users only).
- [QGroundControl User Guide > Sensors](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html#flight_controller_orientation)
