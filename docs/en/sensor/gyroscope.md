# Gyroscope Hardware & Setup

PX4 uses a gyroscope for estimating the vehicle attitude (orientation).

You should not need to attach a gyroscope as a stand-alone external device:

- Most flight controllers, such as those in the [Pixhawk Series](../flight_controller/pixhawk_series.md), include a gyroscope as part of the flight controller's [Inertial Motion Unit (IMU)](https://en.wikipedia.org/wiki/Inertial_measurement_unit).
- Gyroscopes are present as part of an [external INS, ARHS or INS-enhanced GNSS system](../sensor/inertial_navigation_systems.md).

The gyroscope must be calibrated before first use of the vehicle:

- [Gyroscope Calibration](../config/gyroscope.md)
