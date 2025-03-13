# IMU, AHRS та INS

PX4 зазвичай працює на контролерах польоту, які включають в себе ІВП, такі як серія Pixhawk, і об'єднує дані датчиків разом із інформацією ССН (супутникова система навігації) в оцінювачі EKF2 для визначення орієнтації, напрямку, позиції та швидкості транспортного засобу.

However PX4 can also use some INS devices as either sources of raw data, or as an external estimator, replacing the EKF.

Системи, які можуть бути використані у такий спосіб, включають в себе:

- [VectorNav](../sensor/vectornav.md): ІВП/AHRS, ССН/INS, Dual GNSS/INS системи, котрі можуть бути використані як зовнішній INS, або джерело вхідної інформації датчиків.

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
