# Barometers

Barometers measure atmospheric pressure, and are used in drones as altitude sensors.

Most [flight controllers](../flight_controller/index.md) on which PX4 runs include a barometer.
By default PX4 will select the barometer with the highest priority (if any are present), and configure it as a data source for [Height estimation](../advanced_config/tuning_the_ecl_ekf.md#height).
If a sensor fault is detected, PX4 will fall back to the next highest priority sensor.

Generally barometers require no user configuration (or thought)!

## Hardware Options

[Pixhawk standard](../flight_controller/autopilot_pixhawk_standard.md) flight controllers include a barometer, as do [many others](../flight_controller/index.md).

They are also present in other hardware:

- [CUAV NEO 3 Pro GNSS module](https://doc.cuav.net/gps/neo-series-gnss/en/neo-3-pro.html#key-data) ([MS5611](../modules/modules_driver_baro.md#ms5611))
- [RaccoonLab L1 GNSS NEO-M8N](https://docs.raccoonlab.co/guide/gps_mag_baro/)

At time of writing, drivers/parts include: bmp280, bmp388 (and BMP380), dps310, goertek (spl06), invensense (icp10100, icp10111, icp101xx, icp201xx), lps22hb, lps25h, lps33hw, maiertek (mpc2520), mpl3115a2, ms5611, ms5837, tcbp001ta.

Note that the supported barometer part numbers can be inferred from the driver names listed in the [Modules Reference: Baro (Driver)](../modules/modules_driver_baro.md) documentation (and the driver source: [PX4-Autopilot/src/drivers/barometer](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/barometer)).

## PX4 Configuration

Generally barometers require no user configuration.
If needed, you can:

- Enable/Disable barometers as data source for [Height estimation](../advanced_config/tuning_the_ecl_ekf.md#height) using the [EKF2_BARO_CTRL](../advanced_config/parameter_reference.md#EKF2_BARO_CTRL) parameter.
- Change the selection order of barometers using the [CAL_BAROx_PRIO](../advanced_config/parameter_reference.md#CAL_BARO0_PRIO) parameters for each barometer.
- Disable a barometer by setting its [CAL_BAROx_PRIO](../advanced_config/parameter_reference.md#CAL_BARO0_PRIO) value to `0`.

## Baro Auto-Calibration (Developers)

::: tip
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

::: info
GNSS-baro calibration requires an operational GNSS receiver passing all enabled checks in [GNSS_CHECK](../advanced_config/parameter_reference.md#GNSS_CHECK).
Relative calibration must already have completed.
:::

GNSS-baro calibration adjusts baro sensor offsets to align with absolute altitude measurements from the GNSS receiver.
This calibration is controlled by the [SENS_BAR_AUTOCAL](../advanced_config/parameter_reference.md#SENS_BAR_AUTOCAL) parameter (enabled by default).

The algorithm monitors GNSS quality, collects altitude differences over a 2-second filtered window, and verifies stability within 4m tolerance.
Once stable, it uses binary search to calculate pressure offsets that align the baro altitude with the filtered GNSS altitude (0.1m precision), then applies the offset to all sensors and saves the parameters.

While the vehicle stays on the ground and disarmed, the calibration keeps running: a baro drifting away from the GNSS altitude (for example from sensor warm-up or weather changes between boot and takeoff) is recalibrated once its offset, filtered over 5 seconds, exceeds 0.5 m, at most once per minute.
This tracking stops on arming and resumes after landing and disarming.
It also stops until the next reboot once the altitude is set externally (`VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE`, for example the manual position reset of a ground station), as the GNSS altitude is then not trusted.

Notes:

- **GNSS Height Fusion**: Calibration requires GNSS height fusion to be active ([EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL)).
- **Ground Only**: Calibration only runs while landed.
  The first calibration after boot also completes while armed, so arming early does not leave the baro uncalibrated.
  After that the baro is only recalibrated while disarmed.
- **Height Reset**: EKF2 resets its height to each new baro calibration when the baro is the height reference ([EKF2_HGT_REF](../advanced_config/parameter_reference.md#EKF2_HGT_REF)), so the estimated altitude follows the GNSS altitude until arming.
- **Persistence**: Calibration offsets are saved to parameters and persist across reboots.
- **Faulty GNSS Vulnerability**: The baro follows the GNSS altitude for as long as the vehicle is landed and disarmed.
  A wrong GNSS altitude during boot is corrected once the GNSS altitude recovers, but a GNSS altitude that is still faulty at arming is the reference for the flight.
  See [Faulty GNSS Data During Boot](../advanced_config/tuning_the_ecl_ekf.md#faulty-gnss-data-during-boot) for mitigation strategies.

## See Also

- [Baro driver source code](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/barometer)
- [Modules Reference: Baro (Driver)](../modules/modules_driver_baro.md) documentation.
