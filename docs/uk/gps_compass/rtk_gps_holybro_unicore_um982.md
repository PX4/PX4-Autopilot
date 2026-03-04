# Holybro H-RTK Unicore UM982 GPS

The [Holybro H-RTK Unicore UM982 GPS](https://holybro.com/products/h-rtk-um982) is an multi-band high-precision [RTK GNSS System](../gps_compass/rtk_gps.md) launched by Holybro.

![HB-pmw3901-1](../../assets/hardware/gps/holybro-unicore-um982/holybro-unicore-um982-1.jpg)

This module is based on the [Unicore UM982 Chip](https://en.unicorecomm.com/products/detail/24), which supports RTK positioning and dual-antenna heading calculation.

Це означає, що він може генерувати рухому базову лінію визначення курсу/рискання для автопілотів з одним GPS-модулем і двома антенами - магнітометр не потрібен.
Unlike when using a module such as the U-blox F9P, where you would need [two U-blox F9P modules to compute a heading angle](../gps_compass/u-blox_f9p_heading.md), with the Unicore UM982 GPS, you only need one GPS module!

Використання цього GPS замість компаса запобігає виникненню магнітних перешкод, які можуть спричинити неправильні звіти автопілота (на компаси зазвичай впливають двигуни та електричні системи транспортного засобу, а також інші джерела зовнішніх перешкод, такі як металеві конструкції або обладнання).
Це працює, навіть якщо GPS не отримує дані RTCM від стаціонарної станції RTK або сервера NTRIP.
Він підтримує регулювання позиціонування RTK з сантиметровою точністю, системи глобального позиціонування GPS/ГЛОНАСС, Beidou, Galileo і QZSS.

Модуль також містить магнітометр, світлодіод і кнопку захисного перемикача.
Він також слугує як транспортний GPS з RTK-корекцією, з визначенням рухомої базової лінії рискання або без неї, а також як базова станція GPS для надсилання даних RTCM на наземну станцію управління, щоб забезпечити джерело RTK для транспортного засобу за допомогою телеметрії.

Additional technical information can be found at [Holybro Technical Documentation page](https://docs.holybro.com/gps-and-rtk-system/h-rtk-unicore-um982)

## Де купити

- [Holybro Website](https://holybro.com/products/h-rtk-um982)

## Підключення

The module comes with both GH 10-pin & 6-pin cables that are compatible with the GPS1 & GPS2 ports on flight controllers that use the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf), such as [Pixhawk 6x](../flight_controller/pixhawk6x.md) and [Pixhawk 6c](../flight_controller/pixhawk6c.md).

Він також може використовуватися з польотним контролером Cubepilot.
The 10Pin - 6Pin cable allows users to connect the UM982 to `GPS2` port on Cubepilot and Holybro Autopilots.

Модуль можна використовувати з однією антеною або з обома антенами.
Якщо пристрій використовується лише з однією антеною, необхідно підключити правий/первинний роз'єм антени.

## Конфігурація PX4

### Налаштування порту

Модуль Unicore використовує протокол NMEA, розширений деякими власними повідомленнями Unicore.
Швидкість передачі даних (baudrate) для послідовного порту складає 230400.

The following PX4 parameters [must be set](../advanced_config/parameters.md):

- [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD) -> 230400
- [GPS_1_PROTOCOL](../advanced_config/parameter_reference.md#GPS_1_PROTOCOL) -> 6: NMEA

Note, the above parameters assume you are connected to `GPS 1`.
Якщо ви використовуєте інший порт, вам слід використовувати його параметри для налаштування швидкості передачі даних та протоколу.

### Увімкнути Heading/Yaw GPS

Модуль Unicore поставляється з двома антенами, первинною (правий роз'єм) і вторинною (лівий роз'єм), які можна використовувати для отримання даних про рискання від GPS.
Вам потрібно встановити наступні параметри:

- [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL): Set bit 3 (8) to enable dual antenna heading into the yaw estimation.
- [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET): Set heading offset to 0 if the primary antenna is in the front.
  Кут збільшується за годинниковою стрілкою, тому встановіть зміщення на 90 градусів, якщо основна антена знаходиться на правому боці транспортного засобу (а додаткова - на лівому).

### RTK Корекції

RTK працює так само, як і модулі uBlox F9P.
RTCMv3 corrections as sent by QGroundControl from an RTK GPS base station are consumed by the Unicore module, which should then change fix type to `RTK float` or `RTK fixed`.
