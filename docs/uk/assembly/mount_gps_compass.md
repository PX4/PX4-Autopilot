# Монтаж компаса (або GNSS/компаса)

Compass and GNSS/Compass modules should be mounted on the frame as far away from motor/ESC power lines and other sources of electromagnetic interference as possible, and [oriented](#compass-orientation) upright with the direction marker pointing towards the front of the vehicle.
You should also configure PX4 to [set the position](#position) of the receiver relative to the centre-of-gravity (CoG).

Для багатороторних літаків зазвичай компас монтується на підставці, тоді як для фіксованих крил та VTOL компас зазвичай монтується на крилі.

## Орієнтація компаса

The compass should ideally be oriented so that it is upright and the direction marker is pointing towards the front of the vehicle (the default orientation), but if needed can be oriented at multiples of 45° from this attitude (in any axis) as defined in the [standard MAVLink orientations](https://mavlink.io/en/messages/common.html#MAV_SENSOR_ORIENTATION) (these follow the same frame convention as when [orienting the flight controller](../config/flight_controller_orientation.md#calculating-orientation)).

На діаграмі нижче показано напрямний маркер на контролері польоту Pixhawk 4 та компасі.

![Connect compass/GPS to Pixhawk 4](../../assets/flight_controller/pixhawk4/pixhawk4_compass_gps.jpg)

PX4 will automatically detect the orientation for any of these standard orientations during [compass calibration](../config/compass.md) ([by default](../advanced_config/parameter_reference.md#SENS_MAG_AUTOROT)).

Компас також може бути монтуваний в будь-яких інших "власних кутах Ейлера", але в цьому випадку вам доведеться вручну налаштувати орієнтації.
For more information see [Setting the Compass Orientation](../config/flight_controller_orientation.md#setting-the-compass-orientation) in _Flight Controller/Sensor Orientation_.

## Положення

In order to compensate for the relative motion between the receiver and the CoG, you should [configure](../advanced_config/parameters.md) the following parameters to set the offsets: [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z).

Це важливо, оскільки кадр тіла, що оцінюється EKF, буде збігатися з місцезнаходженням модуля GNSS та вважатиме його розміщеним на CoG(ЦМ). Якщо модуль GNSS значно відокремлений від CoG(ЦМ), то обертання навколо CoG(ЦМ) буде інтерпретовано як зміна висоти, що у деяких режимах польоту (наприклад, режим позиції) може призвести до непотрібних корекцій.

It is particularly important if using [RTK GNSS](../advanced/rtk_gps.md) which has centimeter-level accuracy, because if the offsets are not set then GNSS measurements will often be rejected as inconsistent with the current EFK estimate.

:::details
Explanation
For example, if the GNSS module is 10cm above the CoG, and the IMU is located at the GoG, a pitch motion of 1 rad/s will create a GNSS velocity measurement of 10cm/s _even though the CoG isn't moving_.
Якщо точність швидкості приймача GNSS становить 1 см/с, то EKF може перестати довіряти вимірюванням, оскільки вони виглядають невірними (помилкові на 10 разів точніше).
Якщо зсуви визначені, EKF скоригує вимірювання, використовуючи дані гіроскопа.
:::
