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

## Baro Auto-Calibration (Developers)

:::tip
This section documents the automated calibration mechanisms that ensure accurate altitude measurements throughout flight operations.
It is intended primarily for a developer audience who want to understand the underlying mechanisms.
:::

The system implements two complementary calibration approaches that work together to maintain altitude measurement precision.
Both calibrations are initiated at the beginning after a system boot.
Relative calibration is performed first, followed by GNSS-barometric calibration.

### Relative Calibration

Relative baro calibration is **always enabled** and operates automatically during system initialization.
This calibration establishes offset corrections for all secondary baro sensors relative to the primary (selected) sensor.

This calibration:

- Eliminates altitude jumps when switching between baro sensors during flight.
- Ensures consistent altitude readings across all available baro sensors.
- Maintains seamless sensor redundancy and failover capability.

### GNSS-Baro Calibration

:::info
GNSS-baro calibration requires an operational GNSS receiver with vertical accuracy (EPV) ≤ 8 meters.
Relative calibration must already have completed.
:::

GNSS-baro calibration adjusts baro sensor offsets to align with absolute altitude measurements from the GNSS receiver.
This calibration is controlled by the [SENS_BAR_AUTOCAL](../advanced_config/parameter_reference.md#SENS_BAR_AUTOCAL) parameter (enabled by default).

The algorithm monitors GNSS quality, collects altitude differences over a 2-second filtered window, and verifies stability within 4m tolerance.
Once stable, it uses binary search to calculate pressure offsets that align baro altitude with GNSS altitude (0.1m precision), then applies the offset to all sensors and saves the parameters.

Примітки:

- **EKF Independence**: GNSS-baro calibration operates independently of EKF2 altitude fusion settings.
- **Execution Timing**: Calibration runs even when [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) altitude fusion is disabled.
- **One-Time Process**: Each calibration session completes once per system startup.
- **Persistence**: Calibration offsets are saved to parameters and persist across reboots.
- **Faulty GNSS Vulnerability**: If GNSS data is faulty during boot, the calibration will use incorrect altitude reference.
  See [Faulty GNSS Data During Boot](../advanced_config/tuning_the_ecl_ekf.md#faulty-gnss-data-during-boot) for mitigation strategies.

## Дивіться також

- [Baro driver source code](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/barometer)
- [Modules Reference: Baro (Driver)](../modules/modules_driver_baro.md) documentation.
