# Sensor Hardware & Setup

This section describes the mandatory and optional sensors and their setup/configuration.

## 개요

PX4-based systems use sensors to estimate vehicle state, which is needed for stabilization and to enable autonomous control.
Vehicle state information includes: position/altitude, heading, speed, airspeed, orientation (attitude), rates of rotation in different directions, battery level, and so on.

PX4 _minimally requires_ a gyroscope, accelerometer, magnetometer (compass), and barometer to measure the above states.
Fixed-wing and VTOL-vehicles _should_ also include an airspeed sensor.
A GPS or other positioning system is needed to enable all automatic modes, and some manual/assisted modes.

[Pixhawk Series](../flight_controller/pixhawk_series.md) flight controllers already have the minimum set of sensors (other controller platforms often do too).
Additional/external sensors can be attached to the controller — an external GPS and compass are recommended, along with an airspeed sensor for VTOL and Fixed wing vehicles.

## Sensor Topics

Mandatory (included in Pixhawk series FCs):

- [Accelerometer](../sensor/accelerometer.md) — Measures changing acceleration.
- [Gyroscope](../sensor/gyroscope.md) — Measures orientation.
- [Magnetometer (Compass)](../gps_compass/magnetometer.md) — Measures heading/direction.
  External compass recommended!
- [Barometers](../sensor/barometer.md) — Measures altitude (via air pressure).

Recommended:

- [Airspeed Sensors](../sensor/airspeed.md) — Measures airspeed.
  Highly recommended for VTOL and Fixed-wing as they are the only mechanism to detect stall.
- [GNSS (GPS)](../gps_compass/index.md) — Measures global position.
  Needed for missions, and some other automatic and manual/assisted modes.
- [RTK GNSS (GPS)](../gps_compass/rtk_gps.md) — GNSS with centimetre-level accuracy.
  Some setups also allow heading to be determine from GNSS rather than a magnetometer.

Optional:

- [Distance Sensors (Rangefinders)](../sensor/rangefinders.md) — Measures distance to target.
  Aids landing, object avoidance, and terrain following.
- [Optical Flow](../sensor/optical_flow.md) — Estimates velocity using a downward facing camera and a downward facing distance sensor.
  Enables a more accurate position lock than GPS alone, and can be used indoors when no GPS signal is available.
- [Tachometers (Revolution Counters)](../sensor/tachometers.md) — Only used for logging.

Other optional:

- [IMU/Compass Factory Calibration](../advanced_config/imu_factory_calibration.md) — Save calibration settings to persistent storage.
- [Sensor Thermal Compensation](../advanced_config/sensor_thermal_calibration.md) — Compensate sensors for temperature variations.
