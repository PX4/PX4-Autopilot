# Барометр

Барометри вимірюють атмосферний тиск і використовуються в дронах як датчики висоти.

Most [flight controllers](../flight_controller/index.md) on which PX4 runs include a barometer.
By default PX4 will select the barometer with the highest priority (if any are present), and configure it as a data source for [Height estimation](../advanced_config/tuning_the_ecl_ekf.md#height).
Якщо виявлено несправність датчика, PX4 перейде на наступний за пріоритетом датчик.

Зазвичай барометри не потребують налаштування користувачем (або думки)!

## Варіанти устаткування

[Pixhawk standard](../flight_controller/autopilot_pixhawk_standard.md) flight controllers include a barometer, as do [many others](../flight_controller/index.md).

Вони також присутні в іншому обладнанні:

- [CUAV NEO 3 Pro GNSS module](https://doc.cuav.net/gps/neo-series-gnss/en/neo-3-pro.html#key-data) ([MS5611](../modules/modules_driver_baro.md#ms5611))
- [RaccoonLab L1 GNSS NEO-M8N](https://raccoonlab.co/tproduct/360882105-258620719271-cyphal-and-dronecan-gnss-m8n-magnetomete)

На момент написання водії/частини включають: bmp280, bmp388 (та BMP380), dps310, goertek (spl06), invensense (icp10100, icp10111, icp101xx, icp201xx), lps22hb, lps25h, lps33hw, maiertek (mpc2520), mpl3115a2, ms5611, ms5837, tcbp001ta.

Note that the supported barometer part numbers can be inferred from the driver names listed in the [Modules Reference: Baro (Driver)](../modules/modules_driver_baro.md) documentation (and the driver source: [PX4-Autopilot/src/drivers/barometer](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/barometer)).

## Конфігурація PX4

Зазвичай барометри не потребують налаштування користувачем.
Якщо потрібно, ви можете:

- Enable/Disable barometers as data source for [Height estimation](../advanced_config/tuning_the_ecl_ekf.md#height) using the [EKF2_BARO_CTRL](../advanced_config/parameter_reference.md#EKF2_BARO_CTRL) parameter.
- Change the selection order of barometers using the [CAL_BAROx_PRIO](../advanced_config/parameter_reference.md#CAL_BARO0_PRIO) parameters for each barometer.
- Disable a barometer by setting its [CAL_BAROx_PRIO](../advanced_config/parameter_reference.md#CAL_BARO0_PRIO) value to `0`.

## Калібрування

Барометри не потребують калібрування.

<!-- Notes:
- Absolute value isn't important since we just use the difference in altitude between "now" and the value when initializing EKF2
- There is usually a scale factor error but it's compensated by the GNSS altitude using a bias estimator in EKF2 (we don't provide a way to calibrate that). This method is fine as long as the height change of the drone isn't too fast (below 200-300km/h probably; don't have real data on that).
- The baro readings can be corrected using a param SENS_BARO_QNH (https://en.wikipedia.org/wiki/Altimeter_setting) parameter, but again, it is only necessary to adjust it if the absolute barometric altitude is required by the pilot.
-->

## Інформація для розробників

- [Baro driver source code](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/barometer)
- [Modules Reference: Baro (Driver)](../modules/modules_driver_baro.md) documentation.
