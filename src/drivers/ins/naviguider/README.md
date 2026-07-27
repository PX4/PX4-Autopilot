# PNI Naviguider INS Driver
## Quick Start
1. Build the firmware with the driver enabled
    1. `$ cd crab-px4-autopilot`
    2. `$ make [board] buildconfig`
    3. Navigate into `drivers` then `Inertial Navigation Systems (INS)` and enable `naviguider`
    4. Quit menuconfig, save the changes, let the firmware finish building.
    5. Flash your custom build of PX4 to the autopilot.
2. Set required parameters in your PX4 autopilot
    * `SENS_EN_NG`: Enable the Naviguider driver.
    * `SENS_NG_BUS`: The I2C bus which is connected to the Naviguider module.
    * `SENS_NG_ADDR`: The I2C address which the Naviguider uses.
    * `SENS_NG_MODE`: Whether to override the PX4 EKF with the Naviguider's orientation (INS mode, `1`) or feed the sensor measurements into the existing EKF (Sensors Only mode, `0`).
3. Set sensor priorities (for Sensors Only mode) to use Naviguider sensors over interal ones.
    * Parameters take the form of `CAL_[sensor][num]_PRIO`, e.g. `CAL_ACC0_PRIO`, `CAL_GYRO2_PRIO`, `CAL_MAG3_PRIO`, `CAL_BARO1_PRIO`
    * Higher value means higher priority, with 100 being the maximum and 0 being disabled.
4. Tune PID gains.
    * The PID gains will likely be too high for the lower sample rate that an external INS can provide.
## Sensors and Outputs Provided
* 3 Axis Accelerometer
* 3 Axis Gyroscope
* 3 Axis Magnetometer
* Barometer
* Quaternion Attitude Estimate
    * Based on the Accel, Gyro, and Mag after fusion and magnetic disturbance rejection.
    * Defines the attitude in the NED frame, similar to the `vehicle_attitude` uORB message.
* Synthetic Magnetometer
    * Created from the Attitude Estimate and based on the last reading of the magnetic field while stationary and disarmed.
    * It will appear as a separate magnetometer on the same I2C bus but with device ID value that is one more than the Naviguider's device ID.
## Functionality Checks and Controls
un these commands in the MAVLink console to check for basic functionality.
* `naviguider status` to check if the Naviguider driver is running.
* `naviguider [stop,start]` to manually stop and start the Naviguider driver.
* `uorb top` to see how many publishers exist for each message type and how often the messages are published.
* `listener sensor_[accel,gyro,mag,baro]` to check for reasonable values coming from the Naviguider driver.
* `listener vehicle_attitude` if in INS Mode or `listener external_ins_attitude` for Sensors Only Mode to check the attitude quaternion coming from the Naviguider driver.
* `listener vehicle_[imu,magnetometer]` to check for device IDs that match the Naviguider's sensor device IDs, indicating that the PX4 sensors module is using the Naviguider sensors over other sensors that may be present.
* `listener estimator_status` to check if the `[accel,gyro,baro,mag]_device_id` values match the Naviguider's sensor device IDs, indicating that the Naviguider sensors are being used in the EKF2 module (assuming it's running in Sensors Only Mode, INS Mode will disable EKF2 entirely).
* `listener sensors_status_imu` to check the priority, consistency, health, etc. of all the IMUs and Gyros.
* `listener sensors_status_[baro,mag]` to check the priority, consistency, health, etc. of all the barometers and magnetometers.
## Extra Parameters
* `NG_ROT`: The rotation of the Naviguider relative to the autopilot body frame. See `src/lib/conversion/rotation.h` for the `Rotation` enum values.
* `NG_RATE_*`: The desired output rates for `ACC`, `GYRO`, `MAG`, `BARO`, and `ROTVEC`. Note that the values set here will result in actual output rates that are either the maximum capable by the sensor or at least as fast as desired but no more than twice the desired rate.
## Code Architecture
### `naviguider_structs.hpp`
The set of data structures and enums that match what are output and used by the Naviguider.
### `naviguider_consts.hpp`
The constant values used by the Naviguider for fields such as FIFO registers and message IDs.
### `naviguider.hpp`
The definition of the Naviguider driver class.
### `naviguider.cpp`
The implementation of the Naviguider driver class.
### `naviguider_main.cpp`
The start point for the Naviguider driver module. It primarily takes care of module lifetime tasks such as starting and stopping.
