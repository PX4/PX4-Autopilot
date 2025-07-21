# Accelerometer Hardware & Setup

PX4 uses accelerometer data for velocity estimation.

You should not need to attach an accelometer as a stand-alone external device:

- Most flight controllers, such as those in the [Pixhawk Series](../flight_controller/pixhawk_series.md), include an accelerometer as part of the flight controller's [Inertial Motion Unit (IMU)](https://en.wikipedia.org/wiki/Inertial_measurement_unit).
- Gyroscopes are present as part of an [external INS, ARHS or INS-enhanced GNSS system](../sensor/inertial_navigation_systems.md).

The accelerometer must be calibrated before first use of the vehicle:

- [Accelerometer Calibration](../config/accelerometer.md)
