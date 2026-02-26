# IMU, AHRS та INS

PX4 зазвичай працює на контролерах польоту, які включають в себе ІВП, такі як серія Pixhawk, і об'єднує дані датчиків разом із інформацією ССН (супутникова система навігації) в оцінювачі EKF2 для визначення орієнтації, напрямку, позиції та швидкості транспортного засобу.

However PX4 can also use some INS devices as either sources of raw data, or as an external estimator, replacing EKF2.

## Supported INS Systems

INS systems that can be used as a replacement for EKF2 in PX4:

- [InertialLabs](../sensor/inertiallabs.md)
- [MicroStrain](../sensor/microstrain.md): Includes VRU, AHRS, INS, and GNSS/INS devices.
- [SBG Systems](../sensor/sbgecom.md): IMU/AHRS, GNSS/INS, Dual GNSS/INS systems that can be used as an external INS or as a source of raw sensor data.
- [VectorNav](../sensor/vectornav.md): ІВП/AHRS, ССН/INS, Dual GNSS/INS системи, котрі можуть бути використані як зовнішній INS, або джерело вхідної інформації датчиків.

## PX4 Firmware

The driver module for your INS system may not be included in the PX4 firmware for your flight controller by default.

You can check by searching the [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6c/default.px4board#L25) configuration file for your target board for either:

- `CONFIG_COMMON_INS`, which includes drivers for [all INS systems](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/ins/Kconfig).
- The key for the particular INS system you are using, such as:
  - `CONFIG_DRIVERS_INS_ILABS`
  - `CONFIG_DRIVERS_INS_MICROSTRAIN`
  - `CONFIG_DRIVERS_INS_VECTORNAV`

If the required key is not present you can include the module in firmware by adding the key to the `default.px4board` file, or using the [kconfig board configuration tool](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) and then select the driver you want (`Drivers -> INS`).
Note that if you're working on a flight controller where flash memory is limited, you're better off installing just the modules you need.

You will then need to rebuild the firmware.

## Словник

### Інерційний вимірювальний пристрій (ІВП)

An IMU is a device that contains a 3-axis accelerometer (motion sensor), 3-axis gyroscope (rotation sensor), and sometimes a magnetometer (compass).
The gyroscope and accelerometer provide raw sensor data that allow measurement of a system's angular rate and acceleration.
Магнетометр, якщо він є, надає дані сенсора, які можуть бути використані для визначення напрямку, в якому рухається транспортний засіб.

### Система посилки заголовку орієнтації (AHRS)

Система AHRS, яка включає як ІВП, так і систему обробки, яка може надавати інформацію про положення та напрямок на увагу з первинних даних ІВП.

### Система інерціальної навігації (INS)

INS є навігаційним пристроєм, який використовує акселерометри, гіроскопи, можливо, магнітометри та комп'ютер для розрахунку положення, швидкості та швидкості руху об'єкта без необхідності зовнішніх посилань.
Фактично це AHRS, який також включає оцінку положення/швидкості.

## Подальша інформація

- [Що таке інерційна навігаційна система?](https://www.vectornav.com/resources/inertial-navigation-articles/what-is-an-ins) (VectorNav)
- [Посібник з інерціальної навігації](https://www.vectornav.com/resources/inertial-navigation-primer) (VectorNav)
