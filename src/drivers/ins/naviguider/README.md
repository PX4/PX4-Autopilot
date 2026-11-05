# PNI Naviguider INS Driver

Driver for the PNI Naviguider INS module over I2C. It publishes accel, gyro, mag, and baro measurements, plus an attitude quaternion that can either feed the PX4 EKF or replace it.

## Quick Start

1. Build the firmware with the driver enabled

    ```sh
    cd PX4-Autopilot          # or wherever your checkout lives
    make [board] boardconfig  # e.g. make px4_fmu-v6x_default boardconfig
    ```

    1. Navigate into `drivers` then `Inertial Navigation Systems (INS)` and enable `naviguider`
    2. Quit menuconfig, save the changes, let the firmware finish building.
    3. Flash your custom build of PX4 to the autopilot.
2. Set the startup parameters in your PX4 autopilot — at minimum `SENS_EN_NG`, `SENS_NG_BUS`, `SENS_NG_ADDR`, and `SENS_NG_MODE`. See [Parameters](#parameters) for defaults and ranges. These are read once at startup, so **reboot after changing them**.
3. Set sensor priorities (for Sensors Only mode) to use Naviguider sensors over internal ones.
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
    * Created from the Attitude Estimate and based on the last reading of the magnetic field while stationary and disarmed, or from the `NG_SYN_MAG_*` parameters.
    * It appears as a separate magnetometer on the same I2C bus, registered under `SENS_NG_ADDR + 1` so that it gets its own device ID. Make sure nothing else on that bus uses the next address up.

## Parameters

### Startup parameters

Read once when the driver starts. Change them, then reboot.

| Parameter | Default | Range | Description |
| --- | --- | --- | --- |
| `SENS_EN_NG` | `1` (enabled) | `0`/`1` | Enable the Naviguider driver. `rc.sensors` starts the driver at boot when this is `1`. |
| `SENS_NG_BUS` | `2` | `0`–`5` | I2C bus which is connected to the Naviguider module. |
| `SENS_NG_ADDR` | `40` (`0x28`) | `1`–`127` | 7-bit I2C address the Naviguider uses, **in decimal**. |
| `SENS_NG_MODE` | `0` | `0`/`1` | `0` = Sensors Only, feed measurements into the existing EKF. `1` = INS, override the PX4 EKF with the Naviguider's orientation. |
| `NG_ROT` | `0` | `0`–`100` | Rotation of the Naviguider relative to the autopilot body frame. See the `Rotation` enum in `src/lib/conversion/rotation.h`. |

### Runtime parameters

Picked up while the driver is running, within about a second of the change.

| Parameter | Default | Range | Description |
| --- | --- | --- | --- |
| `NG_RATE_ACC` | `200` | `0`–`400` | Requested accelerometer output rate, Hz. |
| `NG_RATE_GYRO` | `200` | `0`–`400` | Requested gyroscope output rate, Hz. |
| `NG_RATE_MAG` | `35` | `0`–`125` | Requested magnetometer output rate, Hz. |
| `NG_RATE_BARO` | `5` | `0`–`50` | Requested barometer output rate, Hz. |
| `NG_RATE_ROTVEC` | `100` | `0`–`400` | Requested rotation vector (attitude) output rate, Hz. |
| `NG_SYN_MAG_MODE` | `0` | `0`/`1` | Where the synthetic magnetometer gets its local field. `0` = measure it from the Naviguider's own mag and attitude while disarmed and stationary. `1` = take it from `NG_SYN_MAG_N/E/D`. |
| `NG_SYN_MAG_N` | `0.0` | — | Local magnetic field North component, gauss. Used when `NG_SYN_MAG_MODE` is `1`. |
| `NG_SYN_MAG_E` | `0.0` | — | Local magnetic field East component, gauss. |
| `NG_SYN_MAG_D` | `0.0` | — | Local magnetic field Down component, gauss. |

The `NG_RATE_*` values are requests, not guarantees. You get a rate at least as fast as you asked for and no more than twice as fast, or the sensor's maximum if you asked for more than it can do. Check what you actually got with `perf` — the driver keeps interval counters for accel, gyro, mag, and baro.

## Functionality Checks and Controls

Run these commands in the MAVLink console to check for basic functionality.

* `naviguider status` to check if the Naviguider driver is running.
* `naviguider [stop,start]` to manually stop and start the Naviguider driver.
* `uorb top` to see how many publishers exist for each message type and how often the messages are published.
* `listener sensor_[accel,gyro,mag,baro]` to check for reasonable values coming from the Naviguider driver.
* `listener vehicle_attitude` if in INS Mode or `listener external_ins_attitude` for Sensors Only Mode to check the attitude quaternion coming from the Naviguider driver.
* `listener vehicle_[imu,magnetometer]` to check for device IDs that match the Naviguider's sensor device IDs, indicating that the PX4 sensors module is using the Naviguider sensors over other sensors that may be present.
* `listener estimator_status` to check if the `[accel,gyro,baro,mag]_device_id` values match the Naviguider's sensor device IDs, indicating that the Naviguider sensors are being used in the EKF2 module (assuming it's running in Sensors Only Mode, INS Mode will disable EKF2 entirely).
* `listener sensors_status_imu` to check the priority, consistency, health, etc. of all the IMUs and Gyros.
* `listener sensors_status_[baro,mag]` to check the priority, consistency, health, etc. of all the barometers and magnetometers.

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
