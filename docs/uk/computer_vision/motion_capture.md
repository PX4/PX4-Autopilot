# Motion Capture (MoCap)

Захоплення руху (MoCap) — це техніка [комп’ютерного бачення](https://en.wikipedia.org/wiki/Computer_vision) для оцінки 3D _pose_ (положення та орієнтації) транспортного засобу за допомогою механізму позиціонування, який є _зовнішнім_ до засобу.
Він зазвичай використовується для навігації транспортним засобом у ситуаціях, коли GPS відсутній (наприклад, у приміщенні), і надає положення відносно _локальної_ системи координат.

Системи _MoCap_ найчастіше виявляють рух за допомогою інфрачервоних камер, але також можна використовувати інші типи камер, Lidar або Ultra Wideband (UWB).

:::info
_MoCap_ концептуально схожий на [Visual Inertial Odometry (VIO)](../computer_vision/visual_inertial_odometry.md).
Основна відмінність полягає в тому, що у VIO система бачення працює на транспортному засобі та додатково використовує IMU транспортного засобу для надання інформації про швидкість.
:::

## MoCap Ресурси

Для отримання інформації про MoCap, перегляньте:

- [Використання Vision або Motion Capture систем для Position Estimation](../ros/external_position_estimation.md). <!-- bring across info into user guide? -->
- [Політ з Motion Capture (VICON, NOKOV, Optitrack)](../tutorials/motion-capture.md). <!-- bring across info into user guide? -->
- [Using PX4's Navigation Filter (EKF2) > External Vision System](../advanced_config/tuning_the_ecl_ekf.md#external-vision-system)
