# IMU, AHRS, and INS

PX4 typically runs on flight controllers that include an IMU, such as the Pixhawk series, and fuse the sensor data along with GNSS information in the EKF2 estimator to determine vehicle attitude, heading, position, and velocity.

However PX4 can also use some INS devices as either sources of raw data, or as an external estimator, replacing the EKF.

Systems that can be used in this way include:

- [VectorNav](../sensor/vectornav.md): IMU/AHRS, GNSS/INS, Dual GNSS/INS systems that can be used as an external INS or as a source of raw sensor data.

## Glossary

### Inertial Measurement Unit (IMU)

An IMU is a device that contains a 3-axis accelerometer (motion sensor), 3-axis gyroscope (rotation sensor), and sometimes a magnetometer (compass).
The gyroscope and accelerometer provide raw sensor data that allow measurement of a system's angular rate and acceleration.
The magnetometer, if present, provides sensor data that can be used to provide the direction that the vehicle is facing/heading.

### Attitude Heading Reference System (AHRS)

An AHRS system that includes both an IMU and a processing system that can provide attitude and heading information from the IMUs raw data.

### Inertial Navigation System (INS)

An INS is a navigation device that uses accelerometers, gyroscopes, possibly magnetometers, and a computer to calculate the attitude, position, and velocity of a moving object without the need for external references.
Essentially it is an AHRS that also includes position/velocity estimation.

## 更多信息

- [What is an Inertial Navigation System?](https://www.vectornav.com/resources/inertial-navigation-articles/what-is-an-ins) (VectorNav)
- [Inertial Navigation Primer](https://www.vectornav.com/resources/inertial-navigation-primer) (VectorNav)
