# Inertial Labs INS (All sensors, including INS-U/INS-DU)

Inertial Labs designs and develops [IMU](https://inertiallabs.com/products/imu-inertial-measurement-units/), [AHRS](https://inertiallabs.com/products/ahrs/), [GNSS/INS](https://inertiallabs.com/products/ins-inertial-navigation-systems/) and [other](https://inertiallabs.com/) solutions.
A universal protocol is used for [all Inertial Labs sensors](https://inertiallabs.com/).

![INS-U](../../assets/hardware/sensors/inertial/ilabs-ins-u.png)

Benefits to PX4 users:

- Вища точність оцінки напрямку, крену та кочення
- Більш надійна геоприв'язка GNSS
- Покращена точність позиціонування та установки під час конфлікту супутникового зв'язку
- Продуктивність при динамічних умовах (наприклад, запуски катапультою, операції VTOL, високі g або високі операції з великою кутовою швидкістю)
- Work in different spoofing and jamming conditions

PX4 can use these in a mode that provides only raw sensor output (the default), or as an [external INS](../sensor/inertial_navigation_systems.md) that provides both sensor output and INS data such as position and velocity estimates.
The mode is configurable using a parameter.

## Де купити

[Get technical support or send requests to sales team](https://inertiallabs.com/inertial-labs-inc/contact-inertial-labs-team/).
Recommended sensors:

- [INS-U GNSS/INS](https://inertiallabs.com/ins-u-datasheet): Recommended for fixed-wing systems without hovering, where static heading is not necessary.
- [INS-DU DUAL GNSS/INS](https://inertiallabs.com/ins-du-datasheet): Recommended for multicopter systems where hovering and low dynamics requires the use of static heading.

## Налаштування програмного забезпечення

### Підключення

Connect the sensor to any unused flight controller serial interface, such as a spare `GPS` or `TELEM`.

### Встановлення

The Inertial Labs sensors can be mounted in any orientation.
You can set offsets for coordinate axes in the sensor-configuration software that is provided with your sensor.

## Конфігурація прошивки

### Конфігурація PX4

To use the Inertial Labs driver:

1. Build the firmware with the [ilabs](../modules/modules_driver_ins.md#ilabs) module.

   The module is included by default for many boards.
   You can check by searching for the keys `CONFIG_COMMON_INS` (all INS drivers) and `CONFIG_DRIVERS_INS_ILABS` (ilabs driver) in the [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6c/default.px4board#L25) configuration file for your target board.

   If it is not present, you can add the key to your `default.px4board` file, or include it using the [kconfig board configuration](../hardware/porting_guide_config.md#px4-board-configuration-kconfig): Drivers -> INS -> ilabs.

2. [Set the parameter](../advanced_config/parameters.md) [SENS_ILABS_CFG](../advanced_config/parameter_reference.md#SENS_ILABS_CFG) to the hardware port connected to the sensor, such as a spare `GPS` or `TELEM`.
   Make sure that nothing else is configured to use the port (for more information see [Serial Port Configuration](../peripherals/serial_configuration.md)).

3. Перезавантажте PX4.

4. Налаштуйте водія як зовнішній INS або надайте сирові дані:
   - For external INS, set [ILABS_MODE](../advanced_config/parameter_reference.md#ILABS_MODE) to `INS`.
   - For raw inertial sensors, set [ILABS_MODE](../advanced_config/parameter_reference.md#ILABS_MODE) to `Sensors Only`.

     You can then prioritize inertial labs sensors using [CAL_GYROn_PRIO](../advanced_config/parameter_reference.md#CAL_GYRO0_PRIO), [CAL_ACCn_PRIO](../advanced_config/parameter_reference.md#CAL_ACC0_PRIO), [CAL_BAROn_PRIO](../advanced_config/parameter_reference.md#CAL_BARO0_PRIO), [CAL_MAGn_PRIO](../advanced_config/parameter_reference.md#CAL_MAG0_PRIO), where `n` is the instance number of the IMU component (0, 1, etc.).

     ::: tip
     In most cases the external IMU is the highest-numbered.
     Ви можете отримати список доступних компонентів IMU, використовуючи [`uorb top -1`](../middleware/uorb.md#uorb-top-command), ви можете відрізняти їх за допомогою команди [`listener`](../modules/modules_command.md#listener) та розглядаючи дані чи просто швидкості.

:::

5. Перезавантажте PX4.

Після активації модуль буде виявлено при завантаженні.

## Inertial Labs Sensor Configuration

Perform sensor configuration according to the Interface Control Document (ICD) that comes with each device.
The sensor-configuration software allows you to set the baurate and the data format, and you can also adjust the orientation axes if the sensor wasn't oriented in the normal way.
This process is usually short and takes a few minutes, depending on the sensor and the installation conditions on the vehicle.

## Опубліковані дані

These uORB topics are published:

- [sensor_accel](../msg_docs/SensorAccel.md)
- [датчик_гіроскопа](../msg_docs/SensorGyro.md)
- [sensor_mag](../msg_docs/SensorMag.md)
- [sensor_baro](../msg_docs/SensorBaro.md)
- [sensor_gps](../msg_docs/SensorGps.md)

If enabled as an external INS, publishes:

- [vehicle_local_position](../msg_docs/VehicleLocalPosition.md)
- [vehicle_global_positon](../msg_docs/VehicleGlobalPosition.md)
- [vehicle_attitude](../msg_docs/VehicleAttitude.md)

If enabled as external sensor only:

- `external_ins_local_position`
- `external_ins_global_position`
- `external_ins_attitude`

:::tip
Опубліковані теми можна переглянути за допомогою команди `listener`.
:::
