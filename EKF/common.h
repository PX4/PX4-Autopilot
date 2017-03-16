/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file common.h
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */

namespace estimator
{
struct gps_message {
	uint64_t time_usec;
	int32_t lat;                // Latitude in 1E-7 degrees
	int32_t lon;                // Longitude in 1E-7 degrees
	int32_t alt;                // Altitude in 1E-3 meters (millimeters) above MSL
	uint8_t fix_type;           // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time
	float eph;                  // GPS horizontal position accuracy in m
	float epv;                  // GPS vertical position accuracy in m
	float sacc;                 // GPS speed accuracy in m/s
	float vel_m_s;              // GPS ground speed (m/s)
	float vel_ned[3];           // GPS ground speed NED
	bool vel_ned_valid;         // GPS ground speed is valid
	uint8_t nsats;              // number of satellites used
	float gdop;                 // geometric dilution of precision
};

typedef matrix::Vector<float, 2> Vector2f;
typedef matrix::Vector<float, 3> Vector3f;
typedef matrix::Quaternion<float> Quaternion;
typedef matrix::Matrix<float, 3, 3> Matrix3f;

struct flow_message {
	uint8_t quality;			// Quality of Flow data
	Vector2f flowdata;			// Flow data received
	Vector3f gyrodata;			// Gyro data from flow sensor
	uint32_t dt;				// integration time of flow samples
};

struct ext_vision_message {
	Vector3f posNED;  // measured NED position relative to the local origin (m)
	Quaternion quat;  // measured quaternion orientation defining rotation from NED to body frame
	float posErr;     // 1-Sigma spherical position accuracy (m)
	float angErr;     // 1-Sigma angular error (rad)
};

struct outputSample {
	Quaternion  quat_nominal;	// nominal quaternion describing vehicle attitude
	Vector3f    vel;	// NED velocity estimate in earth frame in m/s
	Vector3f    pos;	// NED position estimate in earth frame in m/s
	uint64_t 	time_us;	// timestamp in microseconds
};

struct imuSample {
	Vector3f    delta_ang;	// delta angle in body frame (integrated gyro measurements)
	Vector3f    delta_vel;	// delta velocity in body frame (integrated accelerometer measurements)
	float       delta_ang_dt;	// delta angle integration period in seconds
	float       delta_vel_dt;	// delta velocity integration period in seconds
	uint64_t    time_us;	// timestamp in microseconds
};

struct gpsSample {
	Vector2f    pos;	// NE earth frame gps horizontal position measurement in m
	float       hgt;	// gps height measurement in m
	Vector3f    vel;	// NED earth frame gps velocity measurement in m/s
	float	    hacc;	// 1-std horizontal position error m
	float	    vacc;	// 1-std vertical position error m
	float       sacc;	// 1-std speed error m/s
	uint64_t    time_us;	// timestamp in microseconds
};

struct magSample {
	Vector3f    mag;	// NED magnetometer body frame measurements
	uint64_t    time_us;	// timestamp in microseconds
};

struct baroSample {
	float       hgt;	// barometer height above sea level measurement in m
	uint64_t    time_us;	// timestamp in microseconds
};

struct rangeSample {
	float       rng;	// range (distance to ground) measurement in m
	uint64_t    time_us;	// timestamp in microseconds
};

struct airspeedSample {
	float       true_airspeed;	// true airspeed measurement in m/s
	float 		eas2tas;		// equivalent to true airspeed factor
	uint64_t    time_us;	// timestamp in microseconds
};

struct flowSample {
	uint8_t  quality; // quality indicator between 0 and 255
	Vector2f flowRadXY; // measured delta angle of the image about the X and Y body axes (rad), RH rotaton is positive
	Vector2f flowRadXYcomp;	// measured delta angle of the image about the X and Y body axes after removal of body rotation (rad), RH rotation is positive
	Vector3f gyroXYZ; // measured delta angle of the inertial frame about the body axes obtained from rate gyro measurements (rad), RH rotation is positive
	float    dt; // amount of integration time (sec)
	uint64_t time_us; // timestamp in microseconds of the integration period mid-point
};

struct extVisionSample {
	Vector3f posNED;  // measured NED position relative to the local origin (m)
	Quaternion quat;  // measured quaternion orientation defining rotation from NED to body frame
	float posErr;     // 1-Sigma spherical position accuracy (m)
	float angErr;     // 1-Sigma angular error (rad)
	uint64_t time_us; // timestamp of the measurement in microseconds
};

// Integer definitions for vdist_sensor_type
#define VDIST_SENSOR_BARO  0	// Use baro height
#define VDIST_SENSOR_GPS   1	// Use GPS height
#define VDIST_SENSOR_RANGE 2	// Use range finder height
#define VDIST_SENSOR_EV    3    // USe external vision

// Bit locations for mag_declination_source
#define MASK_USE_GEO_DECL   (1<<0)  // set to true to use the declination from the geo library when the GPS position becomes available, set to false to always use the EKF2_MAG_DECL value
#define MASK_SAVE_GEO_DECL  (1<<1)  // set to true to set the EKF2_MAG_DECL parameter to the value returned by the geo library
#define MASK_FUSE_DECL      (1<<2)  // set to true if the declination is always fused as an observation to constrain drift when 3-axis fusion is performed

// Bit locations for fusion_mode
#define MASK_USE_GPS    (1<<0)  // set to true to use GPS data
#define MASK_USE_OF     (1<<1)  // set to true to use optical flow data
#define MASK_INHIBIT_ACC_BIAS (1<<2)  // set to true to inhibit estimation of accelerometer delta velocity bias
#define MASK_USE_EVPOS	(1<<3)  // set to true to use external vision NED position data
#define MASK_USE_EVYAW  (1<<4)	// set to true to use exernal vision quaternion data for yaw

// Integer definitions for mag_fusion_type
#define MAG_FUSE_TYPE_AUTO      0   // The selection of either heading or 3D magnetometer fusion will be automatic
#define MAG_FUSE_TYPE_HEADING   1   // Simple yaw angle fusion will always be used. This is less accurate, but less affected by earth field distortions. It should not be used for pitch angles outside the range from -60 to +60 deg
#define MAG_FUSE_TYPE_3D        2   // Magnetometer 3-axis fusion will always be used. This is more accurate, but more affected by localised earth field distortions

// Maximum sensor intervals in usec
#define GPS_MAX_INTERVAL	5e5
#define BARO_MAX_INTERVAL	2e5
#define RNG_MAX_INTERVAL	2e5
#define EV_MAX_INTERVAL		2e5

// bad accelerometer detection and mitigation
#define BADACC_PROBATION	10E6	// Number of usec that accel data declared bad must continuously pass checks to be declared good
#define BADACC_HGT_RESET	1E6	// Number of usec that accel data must continuously fail checks to trigger a height reset
#define BADACC_BIAS_PNOISE_MULT 2.0f	// The delta velocity process noise is multiplied by this value when accel data is declared bad

struct parameters {
	// measurement source control
	int fusion_mode;		// bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
	int vdist_sensor_type;		// selects the primary source for height data
	int sensor_interval_min_ms;	// minimum time of arrival difference between non IMU sensor updates. Sets the size of the observation buffers.

	// measurement time delays
	float mag_delay_ms;		// magnetometer measurement delay relative to the IMU (msec)
	float baro_delay_ms;		// barometer height measurement delay relative to the IMU (msec)
	float gps_delay_ms;		// GPS measurement delay relative to the IMU (msec)
	float airspeed_delay_ms;	// airspeed measurement delay relative to the IMU (msec)
	float flow_delay_ms;		// optical flow measurement delay relative to the IMU (msec) - this is to the middle of the optical flow integration interval
	float range_delay_ms;		// range finder measurement delay relative to the IMU (msec)
	float ev_delay_ms;		// off-board vision measurement delay relative to the IMU (msec)

	// input noise
	float gyro_noise;		// IMU angular rate noise used for covariance prediction (rad/sec)
	float accel_noise;		// IMU acceleration noise use for covariance prediction (m/sec/sec)

	// process noise
	float gyro_bias_p_noise;	// process noise for IMU rate gyro bias prediction (rad/sec**2)
	float accel_bias_p_noise;	// process noise for IMU accelerometer bias prediction (m/sec**3)
	float mage_p_noise;		// process noise for earth magnetic field prediction (Guass/sec)
	float magb_p_noise;		// process noise for body magnetic field prediction (Guass/sec)
	float wind_vel_p_noise;		// process noise for wind velocity prediction (m/sec/sec)
	float terrain_p_noise;		// process noise for terrain offset (m/sec)
	float terrain_gradient;		// gradient of terrain used to estimate process noise due to changing position (m/m)

	// initialisation errors
	float switch_on_gyro_bias;	// 1-sigma gyro bias uncertainty at switch on (rad/sec)
	float switch_on_accel_bias;	// 1-sigma accelerometer bias uncertainty at switch on (m/s**2)
	float initial_tilt_err;		// 1-sigma tilt error after initial alignment using gravity vector (rad)

	// position and velocity fusion
	float gps_vel_noise;		// observation noise for gps velocity fusion (m/sec)
	float gps_pos_noise;		// observation noise for gps position fusion (m)
	float pos_noaid_noise;		// observation noise for non-aiding position fusion (m)
	float baro_noise;		// observation noise for barometric height fusion (m)
	float baro_innov_gate;		// barometric height innovation consistency gate size (STD)
	float posNE_innov_gate;		// GPS horizontal position innovation consistency gate size (STD)
	float vel_innov_gate;		// GPS velocity innovation consistency gate size (STD)
	float hgt_reset_lim;		// The maximum 1-sigma uncertainty in height that can be tolerated before the height state is reset (m)

	// magnetometer fusion
	float mag_heading_noise;	// measurement noise used for simple heading fusion (rad)
	float mag_noise;		// measurement noise used for 3-axis magnetoemeter fusion (Gauss)
	float mag_declination_deg;	// magnetic declination (degrees)
	float heading_innov_gate;	// heading fusion innovation consistency gate size (STD)
	float mag_innov_gate;		// magnetometer fusion innovation consistency gate size (STD)
	int mag_declination_source;	// bitmask used to control the handling of declination data
	int mag_fusion_type;		// integer used to specify the type of magnetometer fusion used

	// airspeed fusion
	float tas_innov_gate;		// True Airspeed Innovation consistency gate size in standard deciation
  	float eas_noise;			// EAS measurement noise standard deviation used for airspeed fusion [m/s]

  	// synthetic sideslip fusion
 	float beta_innov_gate;		// synthetic sideslip innovation consistency gate size in standard deviation (STD)
 	float beta_noise;			// synthetic sideslip noise (rad)
 	float beta_avg_ft_us;		// The average time between synthetic sideslip measurements (usec)

	// range finder fusion
	float range_noise;		// observation noise for range finder measurements (m)
	float range_innov_gate;		// range finder fusion innovation consistency gate size (STD)
	float rng_gnd_clearance;	// minimum valid value for range when on ground (m)
	float rng_sens_pitch;		// Pitch offset of the range sensor (rad). Sensor points out along Z axis when offset is zero. Positive rotation is RH about Y axis.
	float range_noise_scaler;	// scaling from range measurement to noise (m/m)

	// vision position fusion
	float ev_innov_gate;		// vision estimator fusion innovation consistency gate size (STD)

	// optical flow fusion
	float flow_noise;		// observation noise for optical flow LOS rate measurements (rad/sec)
	float flow_noise_qual_min;	// observation noise for optical flow LOS rate measurements when flow sensor quality is at the minimum useable (rad/sec)
	int flow_qual_min;		// minimum acceptable quality integer from  the flow sensor
	float flow_innov_gate;		// optical flow fusion innovation consistency gate size (STD)
	float flow_rate_max;		// maximum valid optical flow rate (rad/sec)

	// these parameters control the strictness of GPS quality checks used to determine uf the GPS is
	// good enough to set a local origin and commence aiding
	int gps_check_mask;     // bitmask used to control which GPS quality checks are used
	float req_hacc;         // maximum acceptable horizontal position error
	float req_vacc;         // maximum acceptable vertical position error
	float req_sacc;         // maximum acceptable speed error
	int req_nsats;          // minimum acceptable satellite count
	float req_gdop;         // maximum acceptable geometric dilution of precision
	float req_hdrift;       // maximum acceptable horizontal drift speed
	float req_vdrift;       // maximum acceptable vertical drift speed

	// XYZ offset of sensors in body axes (m)
	Vector3f imu_pos_body;	// xyz position of IMU in body frame (m)
	Vector3f gps_pos_body;	// xyz position of the GPS antenna in body frame (m)
	Vector3f rng_pos_body;	// xyz position of range sensor in body frame (m)
	Vector3f flow_pos_body;	// xyz position of range sensor focal point in body frame (m)
	Vector3f ev_pos_body;	// xyz position of VI-sensor focal point in body frame (m)

	// output complementary filter tuning
	float vel_Tau;	// velocity state correction time constant (1/sec)
	float pos_Tau;	// postion state correction time constant (1/sec)

	// accel bias learning control
	float acc_bias_lim;		// maximum accel bias magnitude (m/s/s)
	float acc_bias_learn_acc_lim;	// learning is disabled if the magnitude of the IMU acceleration vector is greeater than this (m/sec**2)
	float acc_bias_learn_gyr_lim;	// learning is disabled if the magnitude of the IMU angular rate vector is greeater than this (rad/sec)
	float acc_bias_learn_tc;	// time constant used to control the decaying envelope filters applied to the accel and gyro magnitudes (sec)

	unsigned no_gps_timeout_max;	// maximum time we allow dead reckoning while both gps position and velocity measurements are being
					// rejected before attempting to reset the states to the GPS measurement (usec)
	unsigned no_aid_timeout_max;	// maximum lapsed time from last fusion of measurements that constrain drift before
					// the EKF will report that it is dead-reckoning (usec)

	// Initialize parameter values.  Initialization must be accomplished in the constructor to allow C99 compiler compatibility.
	parameters()
	{
		// measurement source control
		fusion_mode = MASK_USE_GPS;
		vdist_sensor_type = VDIST_SENSOR_BARO;
		sensor_interval_min_ms = 20;

		// measurement time delays
		mag_delay_ms = 0.0f;
		baro_delay_ms = 0.0f;
		gps_delay_ms = 200.0f;
		airspeed_delay_ms = 200.0f;
		flow_delay_ms = 5.0f;
		range_delay_ms = 5.0f;
		ev_delay_ms = 100.0f;

		// input noise
		gyro_noise = 1.5e-2f;
		accel_noise = 3.5e-1f;

		// process noise
		gyro_bias_p_noise = 1.0e-3f;
		accel_bias_p_noise = 3.0e-3f;
		mage_p_noise = 1.0e-3f;
		magb_p_noise = 1.0e-4f;
		wind_vel_p_noise = 1.0e-1f;
		terrain_p_noise = 5.0f;
		terrain_gradient = 0.5f;

		// initialisation errors
		switch_on_gyro_bias = 0.1f;
		switch_on_accel_bias = 0.2f;
		initial_tilt_err = 0.1f;

		// position and velocity fusion
		gps_vel_noise = 5.0e-1f;
		gps_pos_noise = 0.5f;
		pos_noaid_noise = 10.0f;
		baro_noise = 2.0f;
		baro_innov_gate = 5.0f;
		posNE_innov_gate = 5.0f;
		vel_innov_gate = 5.0f;
		hgt_reset_lim = 0.0f;

		// magnetometer fusion
		mag_heading_noise = 3.0e-1f;
		mag_noise = 5.0e-2f;
		mag_declination_deg = 0.0f;
		heading_innov_gate = 2.6f;
		mag_innov_gate = 3.0f;
		mag_declination_source = 7;
		mag_fusion_type = 0;

		// airspeed fusion
		tas_innov_gate = 5.0f;
  		eas_noise = 1.4f;

		// synthetic sideslip fusion
 		beta_innov_gate = 5.0f;
		beta_noise = 0.3f;
 		beta_avg_ft_us = 1000000.0f; //1 Hz

		// range finder fusion
		range_noise = 0.1f;
		range_innov_gate = 5.0f;
		rng_gnd_clearance = 0.1f;
		rng_sens_pitch = 0.0f;
		range_noise_scaler = 0.0f;

		// vision position fusion
		ev_innov_gate = 5.0f;

		// optical flow fusion
		flow_noise = 0.15f;
		flow_noise_qual_min = 0.5f;
		flow_qual_min = 1;
		flow_innov_gate = 3.0f;
		flow_rate_max = 2.5f;

		// GPS quality checks
		gps_check_mask = 21;
		req_hacc = 5.0f;
		req_vacc = 8.0f;
		req_sacc = 1.0f;
		req_nsats = 6;
		req_gdop = 2.0f;
		req_hdrift = 0.3f;
		req_vdrift = 0.5f;

		// XYZ offset of sensors in body axes (m)
		imu_pos_body = {};
		gps_pos_body = {};
		rng_pos_body = {};
		flow_pos_body = {};
		ev_pos_body = {};

		// output complementary filter tuning time constants
		vel_Tau = 0.25f;
		pos_Tau = 0.25f;

		// accel bias state limiting
		acc_bias_lim = 0.4f;
		acc_bias_learn_acc_lim = 25.0f;
		acc_bias_learn_gyr_lim = 3.0f;
		acc_bias_learn_tc = 0.5f;

		// dead reckoning timers
		no_gps_timeout_max = 7e6;
		no_aid_timeout_max = 1e6;

	}
};

struct stateSample {
	Quaternion  quat_nominal; // quaternion defining the rotaton from earth to body frame
	Vector3f    vel;	// NED velocity in earth frame in m/s
	Vector3f    pos;	// NED position in earth frame in m
	Vector3f    gyro_bias;	// delta angle bias estimate in rad
	Vector3f    accel_bias;	// delta velocity bias estimate in m/s
	Vector3f    mag_I;	// NED earth magnetic field in gauss
	Vector3f    mag_B;	// magnetometer bias estimate in body frame in gauss
	Vector2f    wind_vel;	// wind velocity in m/s
};

union fault_status_u {
	struct {
		bool bad_mag_x: 1;	// 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
		bool bad_mag_y: 1;	// 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
		bool bad_mag_z: 1;	// 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
		bool bad_mag_hdg: 1;	// 3 - true if the fusion of the magnetic heading has encountered a numerical error
		bool bad_mag_decl: 1;	// 4 - true if the fusion of the magnetic declination has encountered a numerical error
		bool bad_airspeed: 1;	// 5 - true if fusion of the airspeed has encountered a numerical error
		bool bad_sideslip: 1;	// 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
		bool bad_optflow_X: 1;	// 7 - true if fusion of the optical flow X axis has encountered a numerical error
		bool bad_optflow_Y: 1;	// 8 - true if fusion of the optical flow Y axis has encountered a numerical error
		bool bad_vel_N: 1;	// 9 - true if fusion of the North velocity has encountered a numerical error
		bool bad_vel_E: 1;	// 10 - true if fusion of the East velocity has encountered a numerical error
		bool bad_vel_D: 1;	// 11 - true if fusion of the Down velocity has encountered a numerical error
		bool bad_pos_N: 1;	// 12 - true if fusion of the North position has encountered a numerical error
		bool bad_pos_E: 1;	// 13 - true if fusion of the East position has encountered a numerical error
		bool bad_pos_D: 1;	// 14 - true if fusion of the Down position has encountered a numerical error
		bool bad_acc_bias: 1;	// 15 - true if bad delta velocity bias estimates have been detected
	} flags;
	uint16_t value;

};

// define structure used to communicate innovation test failures
union innovation_fault_status_u {
	struct {
		bool reject_vel_NED: 1;		// 0 - true if velocity observations have been rejected
		bool reject_pos_NE: 1;		// 1 - true if horizontal position observations have been rejected
		bool reject_pos_D: 1;		// 2 - true if true if vertical position observations have been rejected
		bool reject_mag_x: 1;		// 3 - true if the X magnetometer observation has been rejected
		bool reject_mag_y: 1;		// 4 - true if the Y magnetometer observation has been rejected
		bool reject_mag_z: 1;		// 5 - true if the Z magnetometer observation has been rejected
		bool reject_yaw: 1;		// 6 - true if the yaw observation has been rejected
		bool reject_airspeed: 1;	// 7 - true if the airspeed observation has been rejected
		bool reject_sideslip: 1;	// 8 - true if the synthetic sideslip observation has been rejected
		bool reject_hagl: 1;		// 9 - true if the height above ground observation has been rejected
		bool reject_optflow_X: 1;	// 10 - true if the X optical flow observation has been rejected
		bool reject_optflow_Y: 1;	// 11 - true if the Y optical flow observation has been rejected
	} flags;
	uint16_t value;

};

// publish the status of various GPS quality checks
union gps_check_fail_status_u {
	struct {
		uint16_t fix    : 1; // 0 - true if the fix type is insufficient (no 3D solution)
		uint16_t nsats  : 1; // 1 - true if number of satellites used is insufficient
		uint16_t gdop   : 1; // 2 - true if geometric dilution of precision is insufficient
		uint16_t hacc   : 1; // 3 - true if reported horizontal accuracy is insufficient
		uint16_t vacc   : 1; // 4 - true if reported vertical accuracy is insufficient
		uint16_t sacc   : 1; // 5 - true if reported speed accuracy is insufficient
		uint16_t hdrift : 1; // 6 - true if horizontal drift is excessive (can only be used when stationary on ground)
		uint16_t vdrift : 1; // 7 - true if vertical drift is excessive (can only be used when stationary on ground)
		uint16_t hspeed : 1; // 8 - true if horizontal speed is excessive (can only be used when stationary on ground)
		uint16_t vspeed : 1; // 9 - true if vertical speed error is excessive
	} flags;
	uint16_t value;
};

// bitmask containing filter control status
union filter_control_status_u {
	struct {
		uint16_t tilt_align  : 1; // 0 - true if the filter tilt alignment is complete
		uint16_t yaw_align   : 1; // 1 - true if the filter yaw alignment is complete
		uint16_t gps         : 1; // 2 - true if GPS measurements are being fused
		uint16_t opt_flow    : 1; // 3 - true if optical flow measurements are being fused
		uint16_t mag_hdg     : 1; // 4 - true if a simple magnetic yaw heading is being fused
		uint16_t mag_3D      : 1; // 5 - true if 3-axis magnetometer measurement are being fused
		uint16_t mag_dec     : 1; // 6 - true if synthetic magnetic declination measurements are being fused
		uint16_t in_air      : 1; // 7 - true when the vehicle is airborne
		uint16_t wind        : 1; // 8 - true when wind velocity is being estimated
		uint16_t baro_hgt    : 1; // 9 - true when baro height is being fused as a primary height reference
		uint16_t rng_hgt     : 1; // 10 - true when range finder height is being fused as a primary height reference
		uint16_t gps_hgt     : 1; // 11 - true when GPS height is being fused as a primary height reference
		uint16_t ev_pos      : 1; // 12 - true when local position data from external vision is being fused
		uint16_t ev_yaw      : 1; // 13 - true when yaw data from external vision measurements is being fused
		uint16_t ev_hgt      : 1; // 14 - true when height data from external vision measurements is being fused
		uint16_t fuse_beta   : 1; // 15 - true when synthetic sideslip measurements are being fused
	} flags;
	uint16_t value;
};

union ekf_solution_status {
    struct {
	uint16_t attitude           : 1; // 0 - True if the attitude estimate is good
	uint16_t velocity_horiz     : 1; // 1 - True if the horizontal velocity estimate is good
	uint16_t velocity_vert      : 1; // 2 - True if the vertical velocity estimate is good
	uint16_t pos_horiz_rel      : 1; // 3 - True if the horizontal position (relative) estimate is good
	uint16_t pos_horiz_abs      : 1; // 4 - True if the horizontal position (absolute) estimate is good
	uint16_t pos_vert_abs       : 1; // 5 - True if the vertical position (absolute) estimate is good
	uint16_t pos_vert_agl       : 1; // 6 - True if the vertical position (above ground) estimate is good
	uint16_t const_pos_mode     : 1; // 7 - True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
	uint16_t pred_pos_horiz_rel : 1; // 8 - True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
	uint16_t pred_pos_horiz_abs : 1; // 9 - True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
	uint16_t gps_glitch         : 1; // 10 - True if the EKF has detected a GPS glitch
    } flags;
    uint16_t value;
};

}
