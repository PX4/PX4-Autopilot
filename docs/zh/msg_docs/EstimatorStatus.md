---
pageClass: is-wide-page
---

# EstimatorStatus (UORB message)

**TOPICS:** estimator_status

## Fields

| 参数名                                                                                                                                       | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                                  |
| ----------------------------------------------------------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                                                                 | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                                           |
| timestamp_sample                                                                                                     | `uint64`     |                                                                  |            | the timestamp of the raw data (microseconds)                                                                                                                     |
| output_tracking_error                                                                           | `float32[3]` |                                                                  |            | return a vector containing the output predictor angular, velocity and position tracking error magnitudes (rad), (m/s), (m) |
| gps_check_fail_flags                                                       | `uint16`     |                                                                  |            | Bitmask to indicate status of GPS checks - see definition below                                                                                                                     |
| control_mode_flags                                                                              | `uint64`     |                                                                  |            | Bitmask to indicate EKF logic state                                                                                                                                                 |
| filter_fault_flags                                                                              | `uint32`     |                                                                  |            | Bitmask to indicate EKF internal faults                                                                                                                                             |
| pos_horiz_accuracy                                                                              | `float32`    |                                                                  |            | 1-Sigma estimated horizontal position accuracy relative to the estimators origin (m)                                                                             |
| pos_vert_accuracy                                                                               | `float32`    |                                                                  |            | 1-Sigma estimated vertical position accuracy relative to the estimators origin (m)                                                                               |
| hdg_test_ratio                                                                                  | `float32`    |                                                                  |            | low-pass filtered ratio of the largest heading innovation component to the innovation test limit                                                                                    |
| vel_test_ratio                                                                                  | `float32`    |                                                                  |            | low-pass filtered ratio of the largest velocity innovation component to the innovation test limit                                                                                   |
| pos_test_ratio                                                                                  | `float32`    |                                                                  |            | low-pass filtered ratio of the largest horizontal position innovation component to the innovation test limit                                                                        |
| hgt_test_ratio                                                                                  | `float32`    |                                                                  |            | low-pass filtered ratio of the vertical position innovation to the innovation test limit                                                                                            |
| tas_test_ratio                                                                                  | `float32`    |                                                                  |            | low-pass filtered ratio of the true airspeed innovation to the innovation test limit                                                                                                |
| hagl_test_ratio                                                                                 | `float32`    |                                                                  |            | low-pass filtered ratio of the height above ground innovation to the innovation test limit                                                                                          |
| beta_test_ratio                                                                                 | `float32`    |                                                                  |            | low-pass filtered ratio of the synthetic sideslip innovation to the innovation test limit                                                                                           |
| solution_status_flags                                                                           | `uint16`     |                                                                  |            | Bitmask indicating which filter kinematic state outputs are valid for flight control use.                                                                           |
| reset_count_vel_ne                                                         | `uint8`      |                                                                  |            | number of horizontal position reset events (allow to wrap if count exceeds 255)                                                                                  |
| reset_count_vel_d                                                          | `uint8`      |                                                                  |            | number of vertical velocity reset events (allow to wrap if count exceeds 255)                                                                                    |
| reset_count_pos_ne                                                         | `uint8`      |                                                                  |            | number of horizontal position reset events (allow to wrap if count exceeds 255)                                                                                  |
| reset_count_pod_d                                                          | `uint8`      |                                                                  |            | number of vertical position reset events (allow to wrap if count exceeds 255)                                                                                    |
| reset_count_quat                                                                                | `uint8`      |                                                                  |            | number of quaternion reset events (allow to wrap if count exceeds 255)                                                                                           |
| time_slip                                                                                                            | `float32`    |                                                                  |            | cumulative amount of time in seconds that the EKF inertial calculation has slipped relative to system time                                                                          |
| pre_flt_fail_innov_heading                            | `bool`       |                                                                  |            |                                                                                                                                                                                     |
| pre_flt_fail_innov_height                             | `bool`       |                                                                  |            |                                                                                                                                                                                     |
| pre_flt_fail_innov_pos_horiz     | `bool`       |                                                                  |            |                                                                                                                                                                                     |
| pre_flt_fail_innov_vel_horiz     | `bool`       |                                                                  |            |                                                                                                                                                                                     |
| pre_flt_fail_innov_vel_vert      | `bool`       |                                                                  |            |                                                                                                                                                                                     |
| pre_flt_fail_mag_field_disturbed | `bool`       |                                                                  |            |                                                                                                                                                                                     |
| accel_device_id                                                                                 | `uint32`     |                                                                  |            |                                                                                                                                                                                     |
| gyro_device_id                                                                                  | `uint32`     |                                                                  |            |                                                                                                                                                                                     |
| baro_device_id                                                                                  | `uint32`     |                                                                  |            |                                                                                                                                                                                     |
| mag_device_id                                                                                   | `uint32`     |                                                                  |            |                                                                                                                                                                                     |
| health_flags                                                                                                         | `uint8`      |                                                                  |            | Bitmask to indicate sensor health states (vel, pos, hgt)                                                                                                         |
| timeout_flags                                                                                                        | `uint8`      |                                                                  |            | Bitmask to indicate timeout flags (vel, pos, hgt)                                                                                                                |
| mag_inclination_deg                                                                             | `float32`    |                                                                  |            |                                                                                                                                                                                     |
| mag_inclination_ref_deg                                                    | `float32`    |                                                                  |            |                                                                                                                                                                                     |
| mag_strength_gs                                                                                 | `float32`    |                                                                  |            |                                                                                                                                                                                     |
| mag_strength_ref_gs                                                        | `float32`    |                                                                  |            |                                                                                                                                                                                     |

## Constants

| 参数名                                                                                                                                                                                                           | 类型      | 值  | 描述                                                                                                |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | -- | ------------------------------------------------------------------------------------------------- |
| <a href="#GPS_CHECK_FAIL_GPS_FIX"></a> GPS_CHECK_FAIL_GPS_FIX                                                             | `uint8` | 0  | 0 : insufficient fix type (no 3D solution)                     |
| <a href="#GPS_CHECK_FAIL_MIN_SAT_COUNT"></a> GPS_CHECK_FAIL_MIN_SAT_COUNT                            | `uint8` | 1  | 1 : minimum required sat count fail                                               |
| <a href="#GPS_CHECK_FAIL_MAX_PDOP"></a> GPS_CHECK_FAIL_MAX_PDOP                                                           | `uint8` | 2  | 2 : maximum allowed PDOP fail                                                     |
| <a href="#GPS_CHECK_FAIL_MAX_HORZ_ERR"></a> GPS_CHECK_FAIL_MAX_HORZ_ERR                              | `uint8` | 3  | 3 : maximum allowed horizontal position error fail                                |
| <a href="#GPS_CHECK_FAIL_MAX_VERT_ERR"></a> GPS_CHECK_FAIL_MAX_VERT_ERR                              | `uint8` | 4  | 4 : maximum allowed vertical position error fail                                  |
| <a href="#GPS_CHECK_FAIL_MAX_SPD_ERR"></a> GPS_CHECK_FAIL_MAX_SPD_ERR                                | `uint8` | 5  | 5 : maximum allowed speed error fail                                              |
| <a href="#GPS_CHECK_FAIL_MAX_HORZ_DRIFT"></a> GPS_CHECK_FAIL_MAX_HORZ_DRIFT                          | `uint8` | 6  | 6 : maximum allowed horizontal position drift fail - requires stationary vehicle  |
| <a href="#GPS_CHECK_FAIL_MAX_VERT_DRIFT"></a> GPS_CHECK_FAIL_MAX_VERT_DRIFT                          | `uint8` | 7  | 7 : maximum allowed vertical position drift fail - requires stationary vehicle    |
| <a href="#GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR"></a> GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR | `uint8` | 8  | 8 : maximum allowed horizontal speed fail - requires stationary vehicle           |
| <a href="#GPS_CHECK_FAIL_MAX_VERT_SPD_ERR"></a> GPS_CHECK_FAIL_MAX_VERT_SPD_ERR | `uint8` | 9  | 9 : maximum allowed vertical velocity discrepancy fail                            |
| <a href="#GPS_CHECK_FAIL_SPOOFED"></a> GPS_CHECK_FAIL_SPOOFED                                                                                  | `uint8` | 10 | 10 : GPS signal is spoofed                                                        |
| <a href="#GPS_CHECK_FAIL_JAMMED"></a> GPS_CHECK_FAIL_JAMMED                                                                                    | `uint8` | 11 | 11 : GPS signal is jammed                                                         |
| <a href="#CS_TILT_ALIGN"></a> CS_TILT_ALIGN                                                                                                                         | `uint8` | 0  | 0 - true if the filter tilt alignment is complete                                                 |
| <a href="#CS_YAW_ALIGN"></a> CS_YAW_ALIGN                                                                                                                           | `uint8` | 1  | 1 - true if the filter yaw alignment is complete                                                  |
| <a href="#CS_GNSS_POS"></a> CS_GNSS_POS                                                                                                                             | `uint8` | 2  | 2 - true if GNSS position measurements are being fused                                            |
| <a href="#CS_OPT_FLOW"></a> CS_OPT_FLOW                                                                                                                             | `uint8` | 3  | 3 - true if optical flow measurements are being fused                                             |
| <a href="#CS_MAG_HDG"></a> CS_MAG_HDG                                                                                                                               | `uint8` | 4  | 4 - true if a simple magnetic yaw heading is being fused                                          |
| <a href="#CS_MAG_3D"></a> CS_MAG_3D                                                                                                                                 | `uint8` | 5  | 5 - true if 3-axis magnetometer measurement are being fused                                       |
| <a href="#CS_MAG_DEC"></a> CS_MAG_DEC                                                                                                                               | `uint8` | 6  | 6 - true if synthetic magnetic declination measurements are being fused                           |
| <a href="#CS_IN_AIR"></a> CS_IN_AIR                                                                                                                                 | `uint8` | 7  | 7 - true when thought to be airborne                                                              |
| <a href="#CS_WIND"></a> CS_WIND                                                                                                                                                          | `uint8` | 8  | 8 - true when wind velocity is being estimated                                                    |
| <a href="#CS_BARO_HGT"></a> CS_BARO_HGT                                                                                                                             | `uint8` | 9  | 9 - true when baro data is being fused                                                            |
| <a href="#CS_RNG_HGT"></a> CS_RNG_HGT                                                                                                                               | `uint8` | 10 | 10 - true when range finder data is being fused for height aiding                                 |
| <a href="#CS_GPS_HGT"></a> CS_GPS_HGT                                                                                                                               | `uint8` | 11 | 11 - true when GPS altitude is being fused                                                        |
| <a href="#CS_EV_POS"></a> CS_EV_POS                                                                                                                                 | `uint8` | 12 | 12 - true when local position data from external vision is being fused                            |
| <a href="#CS_EV_YAW"></a> CS_EV_YAW                                                                                                                                 | `uint8` | 13 | 13 - true when yaw data from external vision measurements is being fused                          |
| <a href="#CS_EV_HGT"></a> CS_EV_HGT                                                                                                                                 | `uint8` | 14 | 14 - true when height data from external vision measurements is being fused                       |
| <a href="#CS_BETA"></a> CS_BETA                                                                                                                                                          | `uint8` | 15 | 15 - true when synthetic sideslip measurements are being fused                                    |
| <a href="#CS_MAG_FIELD"></a> CS_MAG_FIELD                                                                                                                           | `uint8` | 16 | 16 - true when only the magnetic field states are updated by the magnetometer                     |
| <a href="#CS_FIXED_WING"></a> CS_FIXED_WING                                                                                                                         | `uint8` | 17 | 17 - true when thought to be operating as a fixed wing vehicle with constrained sideslip          |
| <a href="#CS_MAG_FAULT"></a> CS_MAG_FAULT                                                                                                                           | `uint8` | 18 | 18 - true when the magnetometer has been declared faulty and is no longer being used              |
| <a href="#CS_ASPD"></a> CS_ASPD                                                                                                                                                          | `uint8` | 19 | 19 - true when airspeed measurements are being fused                                              |
| <a href="#CS_GND_EFFECT"></a> CS_GND_EFFECT                                                                                                                         | `uint8` | 20 | 20 - true when when protection from ground effect induced static pressure rise is active          |
| <a href="#CS_RNG_STUCK"></a> CS_RNG_STUCK                                                                                                                           | `uint8` | 21 | 21 - true when a stuck range finder sensor has been detected                                      |
| <a href="#CS_GPS_YAW"></a> CS_GPS_YAW                                                                                                                               | `uint8` | 22 | 22 - true when yaw (not ground course) data from a GPS receiver is being fused |
| <a href="#CS_MAG_ALIGNED"></a> CS_MAG_ALIGNED                                                                                                                       | `uint8` | 23 | 23 - true when the in-flight mag field alignment has been completed                               |
| <a href="#CS_EV_VEL"></a> CS_EV_VEL                                                                                                                                 | `uint8` | 24 | 24 - true when local frame velocity data fusion from external vision measurements is intended     |
| <a href="#CS_SYNTHETIC_MAG_Z"></a> CS_SYNTHETIC_MAG_Z                                                                                          | `uint8` | 25 | 25 - true when we are using a synthesized measurement for the magnetometer Z component            |
| <a href="#CS_VEHICLE_AT_REST"></a> CS_VEHICLE_AT_REST                                                                                          | `uint8` | 26 | 26 - true when the vehicle is at rest                                                             |
| <a href="#CS_GPS_YAW_FAULT"></a> CS_GPS_YAW_FAULT                                                                                              | `uint8` | 27 | 27 - true when the GNSS heading has been declared faulty and is no longer being used              |
| <a href="#CS_RNG_FAULT"></a> CS_RNG_FAULT                                                                                                                           | `uint8` | 28 | 28 - true when the range finder has been declared faulty and is no longer being used              |
| <a href="#CS_GNSS_VEL"></a> CS_GNSS_VEL                                                                                                                             | `uint8` | 44 | 44 - true if GNSS velocity measurement fusion is intended                                         |
| <a href="#CS_GNSS_FAULT"></a> CS_GNSS_FAULT                                                                                                                         | `uint8` | 45 | 45 - true if GNSS measurements have been declared faulty and are no longer used                   |
| <a href="#CS_YAW_MANUAL"></a> CS_YAW_MANUAL                                                                                                                         | `uint8` | 46 | 46 - true if yaw has been set manually                                                            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[3] output_tracking_error # return a vector containing the output predictor angular, velocity and position tracking error magnitudes (rad), (m/s), (m)

uint16 gps_check_fail_flags     # Bitmask to indicate status of GPS checks - see definition below
# bits are true when corresponding test has failed
uint8 GPS_CHECK_FAIL_GPS_FIX = 0		# 0 : insufficient fix type (no 3D solution)
uint8 GPS_CHECK_FAIL_MIN_SAT_COUNT = 1		# 1 : minimum required sat count fail
uint8 GPS_CHECK_FAIL_MAX_PDOP = 2		# 2 : maximum allowed PDOP fail
uint8 GPS_CHECK_FAIL_MAX_HORZ_ERR = 3		# 3 : maximum allowed horizontal position error fail
uint8 GPS_CHECK_FAIL_MAX_VERT_ERR = 4		# 4 : maximum allowed vertical position error fail
uint8 GPS_CHECK_FAIL_MAX_SPD_ERR = 5		# 5 : maximum allowed speed error fail
uint8 GPS_CHECK_FAIL_MAX_HORZ_DRIFT = 6		# 6 : maximum allowed horizontal position drift fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_VERT_DRIFT = 7		# 7 : maximum allowed vertical position drift fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR = 8	# 8 : maximum allowed horizontal speed fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_VERT_SPD_ERR = 9	# 9 : maximum allowed vertical velocity discrepancy fail
uint8 GPS_CHECK_FAIL_SPOOFED = 10		# 10 : GPS signal is spoofed
uint8 GPS_CHECK_FAIL_JAMMED = 11		# 11 : GPS signal is jammed

uint64 control_mode_flags	# Bitmask to indicate EKF logic state
uint8 CS_TILT_ALIGN = 0		# 0 - true if the filter tilt alignment is complete
uint8 CS_YAW_ALIGN = 1		# 1 - true if the filter yaw alignment is complete
uint8 CS_GNSS_POS = 2		# 2 - true if GNSS position measurements are being fused
uint8 CS_OPT_FLOW = 3		# 3 - true if optical flow measurements are being fused
uint8 CS_MAG_HDG = 4		# 4 - true if a simple magnetic yaw heading is being fused
uint8 CS_MAG_3D = 5		# 5 - true if 3-axis magnetometer measurement are being fused
uint8 CS_MAG_DEC = 6		# 6 - true if synthetic magnetic declination measurements are being fused
uint8 CS_IN_AIR = 7		# 7 - true when thought to be airborne
uint8 CS_WIND = 8		# 8 - true when wind velocity is being estimated
uint8 CS_BARO_HGT = 9		# 9 - true when baro data is being fused
uint8 CS_RNG_HGT = 10		# 10 - true when range finder data is being fused for height aiding
uint8 CS_GPS_HGT = 11		# 11 - true when GPS altitude is being fused
uint8 CS_EV_POS = 12		# 12 - true when local position data from external vision is being fused
uint8 CS_EV_YAW = 13		# 13 - true when yaw data from external vision measurements is being fused
uint8 CS_EV_HGT = 14		# 14 - true when height data from external vision measurements is being fused
uint8 CS_BETA = 15		# 15 - true when synthetic sideslip measurements are being fused
uint8 CS_MAG_FIELD = 16		# 16 - true when only the magnetic field states are updated by the magnetometer
uint8 CS_FIXED_WING = 17	# 17 - true when thought to be operating as a fixed wing vehicle with constrained sideslip
uint8 CS_MAG_FAULT = 18		# 18 - true when the magnetometer has been declared faulty and is no longer being used
uint8 CS_ASPD = 19		# 19 - true when airspeed measurements are being fused
uint8 CS_GND_EFFECT = 20	# 20 - true when when protection from ground effect induced static pressure rise is active
uint8 CS_RNG_STUCK = 21		# 21 - true when a stuck range finder sensor has been detected
uint8 CS_GPS_YAW = 22		# 22 - true when yaw (not ground course) data from a GPS receiver is being fused
uint8 CS_MAG_ALIGNED = 23	# 23 - true when the in-flight mag field alignment has been completed
uint8 CS_EV_VEL = 24		# 24 - true when local frame velocity data fusion from external vision measurements is intended
uint8 CS_SYNTHETIC_MAG_Z = 25	# 25 - true when we are using a synthesized measurement for the magnetometer Z component
uint8 CS_VEHICLE_AT_REST = 26	# 26 - true when the vehicle is at rest
uint8 CS_GPS_YAW_FAULT = 27	# 27 - true when the GNSS heading has been declared faulty and is no longer being used
uint8 CS_RNG_FAULT = 28		# 28 - true when the range finder has been declared faulty and is no longer being used
uint8 CS_GNSS_VEL = 44		# 44 - true if GNSS velocity measurement fusion is intended
uint8 CS_GNSS_FAULT = 45        # 45 - true if GNSS measurements have been declared faulty and are no longer used
uint8 CS_YAW_MANUAL = 46        # 46 - true if yaw has been set manually

uint32 filter_fault_flags	# Bitmask to indicate EKF internal faults
# 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
# 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
# 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
# 3 - true if the fusion of the magnetic heading has encountered a numerical error
# 4 - true if the fusion of the magnetic declination has encountered a numerical error
# 5 - true if fusion of the airspeed has encountered a numerical error
# 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
# 7 - true if fusion of the optical flow X axis has encountered a numerical error
# 8 - true if fusion of the optical flow Y axis has encountered a numerical error
# 9 - true if fusion of the North velocity has encountered a numerical error
# 10 - true if fusion of the East velocity has encountered a numerical error
# 11 - true if fusion of the Down velocity has encountered a numerical error
# 12 - true if fusion of the North position has encountered a numerical error
# 13 - true if fusion of the East position has encountered a numerical error
# 14 - true if fusion of the Down position has encountered a numerical error
# 15 - true if bad delta velocity bias estimates have been detected
# 16 - true if bad vertical accelerometer data has been detected
# 17 - true if delta velocity data contains clipping (asymmetric railing)

float32 pos_horiz_accuracy # 1-Sigma estimated horizontal position accuracy relative to the estimators origin (m)
float32 pos_vert_accuracy # 1-Sigma estimated vertical position accuracy relative to the estimators origin (m)

float32 hdg_test_ratio # low-pass filtered ratio of the largest heading innovation component to the innovation test limit
float32 vel_test_ratio # low-pass filtered ratio of the largest velocity innovation component to the innovation test limit
float32 pos_test_ratio # low-pass filtered ratio of the largest horizontal position innovation component to the innovation test limit
float32 hgt_test_ratio # low-pass filtered ratio of the vertical position innovation to the innovation test limit
float32 tas_test_ratio # low-pass filtered ratio of the true airspeed innovation to the innovation test limit
float32 hagl_test_ratio # low-pass filtered ratio of the height above ground innovation to the innovation test limit
float32 beta_test_ratio # low-pass filtered ratio of the synthetic sideslip innovation to the innovation test limit

uint16 solution_status_flags # Bitmask indicating which filter kinematic state outputs are valid for flight control use.
# 0 - True if the attitude estimate is good
# 1 - True if the horizontal velocity estimate is good
# 2 - True if the vertical velocity estimate is good
# 3 - True if the horizontal position (relative) estimate is good
# 4 - True if the horizontal position (absolute) estimate is good
# 5 - True if the vertical position (absolute) estimate is good
# 6 - True if the vertical position (above ground) estimate is good
# 7 - True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
# 8 - True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
# 9 - True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
# 10 - True if the EKF has detected a GPS glitch
# 11 - True if the EKF has detected bad accelerometer data

uint8 reset_count_vel_ne # number of horizontal position reset events (allow to wrap if count exceeds 255)
uint8 reset_count_vel_d  # number of vertical velocity reset events (allow to wrap if count exceeds 255)
uint8 reset_count_pos_ne # number of horizontal position reset events (allow to wrap if count exceeds 255)
uint8 reset_count_pod_d  # number of vertical position reset events (allow to wrap if count exceeds 255)
uint8 reset_count_quat   # number of quaternion reset events (allow to wrap if count exceeds 255)

float32 time_slip # cumulative amount of time in seconds that the EKF inertial calculation has slipped relative to system time

bool pre_flt_fail_innov_heading
bool pre_flt_fail_innov_height
bool pre_flt_fail_innov_pos_horiz
bool pre_flt_fail_innov_vel_horiz
bool pre_flt_fail_innov_vel_vert
bool pre_flt_fail_mag_field_disturbed

uint32 accel_device_id
uint32 gyro_device_id
uint32 baro_device_id
uint32 mag_device_id

# legacy local position estimator (LPE) flags
uint8 health_flags		# Bitmask to indicate sensor health states (vel, pos, hgt)
uint8 timeout_flags		# Bitmask to indicate timeout flags (vel, pos, hgt)

float32 mag_inclination_deg
float32 mag_inclination_ref_deg
float32 mag_strength_gs
float32 mag_strength_ref_gs
```

:::
